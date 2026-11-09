/*
 * 文件总体功能：校园机器人室外到室内导航的“地理坐标层”。
 *
 * 输入：
 *   - /handsfree/rtk/gnss（sensor_msgs/NavSatFix）：RTK/GNSS 给出的 WGS-84
 *     纬度、经度和高程；离线测试时由 GIP.bag 发布，实车时由 RTK 驱动发布。
 *
 * 输出：
 *   - /amap_nav/enu_pose：以选定原点为 (0, 0, 0) 的局部 East-North-Up 米制位置；
 *   - /amap_nav/gcj02_fix：本地 WGS-84 -> GCJ-02 转换结果，供未来地图模块使用；
 *   - /amap_nav/enu_origin：记录 ENU 原点的 WGS-84 经纬高；
 *   - /amap_nav/geo_status：初始化、等待 GNSS、设置原点等状态文字；
 *   - /geo_coordinate_layer/set_origin_from_latest：用最近有效 GNSS 重设原点的服务。
 *
 * 原点规则：默认将第一条通过基础检查的 GNSS 数据设为 ENU 原点；也可在 YAML
 * 中配置一个固定的 WGS-84 原点，以便多次实车测试使用同一局部坐标系。
 *
 * 明确不负责：不调用高德 API、不请求路线、不发布 /cmd_vel、不进行避障；不使用
 * RTK 双天线航向；也不发布 geo_enu 与 Super-LIO /lio/odom 之间的 TF。后者需要
 * 后续独立的“GNSS--LIO 偏航与平移对齐”模块估计后才能安全使用。
 */

#include <algorithm>
#include <cmath>
#include <exception>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

namespace amap_outdoor_indoor_nav {
namespace {

// WGS-84 椭球参数。使用它们先转 ECEF、再转 ENU，避免将地球当成完美球体。
constexpr double kWgs84SemiMajorAxisMetres = 6378137.0;
constexpr double kWgs84Flattening = 1.0 / 298.257223563;
constexpr double kWgs84EccentricitySquared =
    kWgs84Flattening * (2.0 - kWgs84Flattening);
constexpr double kPi = 3.14159265358979323846;

struct EcefPoint {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

double DegreesToRadians(double degrees) {
  return degrees * kPi / 180.0;
}

// 将 WGS-84 纬度、经度、椭球高转换到以地心为原点的 ECEF 三维直角坐标。
// ECEF 只在节点内部用作严格 ENU 换算的中间坐标，不会作为 ROS 话题发布。
EcefPoint GeodeticToEcef(double latitude, double longitude, double altitude) {
  const double latitude_rad = DegreesToRadians(latitude);
  const double longitude_rad = DegreesToRadians(longitude);
  const double sin_latitude = std::sin(latitude_rad);
  const double cos_latitude = std::cos(latitude_rad);
  const double sin_longitude = std::sin(longitude_rad);
  const double cos_longitude = std::cos(longitude_rad);
  const double prime_vertical_radius =
      kWgs84SemiMajorAxisMetres /
      std::sqrt(1.0 - kWgs84EccentricitySquared * sin_latitude * sin_latitude);

  EcefPoint point;
  point.x = (prime_vertical_radius + altitude) * cos_latitude * cos_longitude;
  point.y = (prime_vertical_radius + altitude) * cos_latitude * sin_longitude;
  point.z = (prime_vertical_radius * (1.0 - kWgs84EccentricitySquared) + altitude) *
            sin_latitude;
  return point;
}

// 将两个 ECEF 点的差值旋转到“原点处”的 East-North-Up 局部坐标系。
// 这是真正的 WGS-84 局部 ENU；x/y/z 分别对应东/北/上，单位都是米。
void EcefToEnu(const EcefPoint& point, const EcefPoint& origin, double origin_latitude,
               double origin_longitude, double* east, double* north, double* up) {
  const double latitude_rad = DegreesToRadians(origin_latitude);
  const double longitude_rad = DegreesToRadians(origin_longitude);
  const double delta_x = point.x - origin.x;
  const double delta_y = point.y - origin.y;
  const double delta_z = point.z - origin.z;

  const double sin_latitude = std::sin(latitude_rad);
  const double cos_latitude = std::cos(latitude_rad);
  const double sin_longitude = std::sin(longitude_rad);
  const double cos_longitude = std::cos(longitude_rad);

  *east = -sin_longitude * delta_x + cos_longitude * delta_y;
  *north = -sin_latitude * cos_longitude * delta_x -
           sin_latitude * sin_longitude * delta_y + cos_latitude * delta_z;
  *up = cos_latitude * cos_longitude * delta_x + cos_latitude * sin_longitude * delta_y +
        sin_latitude * delta_z;
}

// GCJ-02 偏移模型只适用于中国境内；境外坐标保持 WGS-84 原值。
bool IsOutsideChina(double latitude, double longitude) {
  return !(72.004 <= longitude && longitude <= 137.8347 &&
           0.8293 <= latitude && latitude <= 55.8271);
}

// GCJ-02 算法的纬度偏移中间项。x/y 是相对于中国参考点的经纬度差。
double TransformLatitude(double x, double y) {
  double value = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y;
  value += 0.1 * x * y + 0.2 * std::sqrt(std::abs(x));
  value += (20.0 * std::sin(6.0 * x * kPi) + 20.0 * std::sin(2.0 * x * kPi)) * 2.0 / 3.0;
  value += (20.0 * std::sin(y * kPi) + 40.0 * std::sin(y / 3.0 * kPi)) * 2.0 / 3.0;
  value += (160.0 * std::sin(y / 12.0 * kPi) + 320.0 * std::sin(y * kPi / 30.0)) * 2.0 / 3.0;
  return value;
}

// GCJ-02 算法的经度偏移中间项。x/y 是相对于中国参考点的经纬度差。
double TransformLongitude(double x, double y) {
  double value = 300.0 + x + 2.0 * y + 0.1 * x * x;
  value += 0.1 * x * y + 0.1 * std::sqrt(std::abs(x));
  value += (20.0 * std::sin(6.0 * x * kPi) + 20.0 * std::sin(2.0 * x * kPi)) * 2.0 / 3.0;
  value += (20.0 * std::sin(x * kPi) + 40.0 * std::sin(x / 3.0 * kPi)) * 2.0 / 3.0;
  value += (150.0 * std::sin(x / 12.0 * kPi) + 300.0 * std::sin(x / 30.0 * kPi)) * 2.0 / 3.0;
  return value;
}

// 在节点内部完成 WGS-84 -> GCJ-02 转换；不访问高德，也不需要 API Key。
void Wgs84ToGcj02(double latitude, double longitude, double* gcj_latitude,
                  double* gcj_longitude) {
  if (IsOutsideChina(latitude, longitude)) {
    *gcj_latitude = latitude;
    *gcj_longitude = longitude;
    return;
  }

  constexpr double kSemiMajorAxis = 6378245.0;
  constexpr double kEccentricitySquared = 0.00669342162296594323;
  double delta_latitude = TransformLatitude(longitude - 105.0, latitude - 35.0);
  double delta_longitude = TransformLongitude(longitude - 105.0, latitude - 35.0);
  const double latitude_rad = latitude * kPi / 180.0;
  double magic = std::sin(latitude_rad);
  magic = 1.0 - kEccentricitySquared * magic * magic;
  const double sqrt_magic = std::sqrt(magic);
  delta_latitude = (delta_latitude * 180.0) /
                   ((kSemiMajorAxis * (1.0 - kEccentricitySquared)) /
                    (magic * sqrt_magic) * kPi);
  delta_longitude = (delta_longitude * 180.0) /
                    (kSemiMajorAxis / sqrt_magic * std::cos(latitude_rad) * kPi);
  *gcj_latitude = latitude + delta_latitude;
  *gcj_longitude = longitude + delta_longitude;
}

// 从原始 NMEA 文本中提取 GGA 的定位质量字段。
// 常见约定：0=无定位，1=单点，2=DGPS，4=RTK Fixed，5=RTK Float。
// 该函数只解析质量值，不负责校验 NMEA 校验和；输入可包含一条或多条 NMEA 语句。
bool ParseLatestGgaQuality(const std::string& raw_nmea, int* quality) {
  std::size_t sentence_start = raw_nmea.find('$');
  bool found = false;

  while (sentence_start != std::string::npos) {
    const std::size_t sentence_end = raw_nmea.find_first_of("\r\n", sentence_start);
    const std::string sentence = raw_nmea.substr(
        sentence_start, sentence_end == std::string::npos ? std::string::npos
                                                            : sentence_end - sentence_start);

    // $GPGGA、$GNGGA 等句型的消息类型均位于下标 3..5。
    if (sentence.size() >= 7 && sentence.substr(3, 3) == "GGA") {
      std::vector<std::string> fields;
      std::stringstream stream(sentence);
      std::string field;
      while (std::getline(stream, field, ',')) {
        fields.push_back(field);
      }

      // GGA 第 7 个逗号分隔字段（下标 6）是定位质量。
      if (fields.size() > 6 && !fields[6].empty()) {
        try {
          std::size_t parsed_characters = 0;
          const int candidate = std::stoi(fields[6], &parsed_characters);
          if (parsed_characters == fields[6].size()) {
            *quality = candidate;
            found = true;
          }
        } catch (const std::exception&) {
          // 遇到格式异常的语句时继续扫描其后的 NMEA 语句。
        }
      }
    }

    if (sentence_end == std::string::npos) {
      break;
    }
    sentence_start = raw_nmea.find('$', sentence_end + 1);
  }
  return found;
}

}  // namespace

// 地理坐标层：管理参数、GNSS 输入、ENU 原点、ENU/GCJ-02 输出和重设原点服务。
class GeoCoordinateLayer {
 public:
  // nh_ 用于普通话题；private_nh_("~") 用于读取本节点私有参数，
  // 例如 /geo_coordinate_layer/enu_frame_id。
  GeoCoordinateLayer() : nh_(), private_nh_("~") {}

  // 节点启动阶段：读取配置、创建发布器、确定原点模式，并建立订阅与服务。
  bool Initialise() {
    // 读取 YAML / launch 注入的功能开关与坐标系配置。
    private_nh_.param<std::string>("enu_frame_id", enu_frame_id_, "geo_enu");
    private_nh_.param<std::string>("origin_mode", origin_mode_, "first_valid_fix");
    private_nh_.param<std::string>("quality_mode", quality_mode_, "any");
    private_nh_.param("require_fix_status", require_fix_status_, false);
    private_nh_.param("max_position_covariance", max_position_covariance_, -1.0);
    private_nh_.param<std::string>("nmea_raw_topic", nmea_raw_topic_, "/handsfree/rtk/raw");
    private_nh_.param("nmea_quality_timeout_sec", nmea_quality_timeout_sec_, 1.0);
    private_nh_.param("publish_gcj02", publish_gcj02_, true);
    // gga_min 模式的最低 GGA 质量门限：1=单点，2=差分/SBAS，4=RTK Fixed，5=RTK Float。
    // 室外采集阶段常只能拿到单点(1)或差分(2)，门限取 1 可在放宽的同时仍拒绝无定位(0)。
    private_nh_.param("min_gga_quality", min_gga_quality_, 1);

    if (quality_mode_ != "any" && quality_mode_ != "not_no_fix" &&
        quality_mode_ != "rtk_class" && quality_mode_ != "rtk_fixed_only" &&
        quality_mode_ != "gga_min") {
      ROS_FATAL("~quality_mode must be one of any, not_no_fix, rtk_class, rtk_fixed_only, "
                "gga_min; got '%s'",
                quality_mode_.c_str());
      return false;
    }
    if (min_gga_quality_ < 1 || min_gga_quality_ > 8) {
      ROS_FATAL("~min_gga_quality must be in [1, 8], got %d", min_gga_quality_);
      return false;
    }
    if (nmea_quality_timeout_sec_ <= 0.0) {
      ROS_FATAL("~nmea_quality_timeout_sec must be positive, got %.3f",
                nmea_quality_timeout_sec_);
      return false;
    }

    // 读取输出话题名称。默认值与 config/geo_coordinate_layer.yaml 保持一致。
    std::string enu_pose_topic;
    std::string gcj02_fix_topic;
    std::string origin_topic;
    std::string status_topic;
    private_nh_.param<std::string>("enu_pose_topic", enu_pose_topic, "/amap_nav/enu_pose");
    private_nh_.param<std::string>("gcj02_fix_topic", gcj02_fix_topic, "/amap_nav/gcj02_fix");
    private_nh_.param<std::string>("origin_topic", origin_topic, "/amap_nav/enu_origin");
    private_nh_.param<std::string>("status_topic", status_topic, "/amap_nav/geo_status");

    // 创建输出发布器。origin_pub_ 使用 latched 模式，后启动的订阅者也能收到原点。
    enu_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(enu_pose_topic, 20);
    origin_pub_ = nh_.advertise<sensor_msgs::NavSatFix>(origin_topic, 1, true);
    status_pub_ = nh_.advertise<std_msgs::String>(status_topic, 10, true);
    if (publish_gcj02_) {
      gcj02_pub_ = nh_.advertise<sensor_msgs::NavSatFix>(gcj02_fix_topic, 20);
    }

    // 原点有三种来源：
    // 1) parameter：使用 YAML 中预先测量并保存的固定经纬高；
    // 2) first_valid_fix：等第一条有效 GNSS 数据到来后再设原点。
    // 3) service：等待操作者确认定位质量后，调用服务手动设原点。
    if (origin_mode_ == "parameter") {
      double latitude = 0.0;
      double longitude = 0.0;
      double altitude = 0.0;
      if (!private_nh_.getParam("origin_latitude", latitude) ||
          !private_nh_.getParam("origin_longitude", longitude)) {
        ROS_FATAL("origin_mode=parameter requires ~origin_latitude and ~origin_longitude");
        return false;
      }
      private_nh_.param("origin_altitude", altitude, 0.0);
      if (!SetOrigin(latitude, longitude, altitude, "parameter")) {
        return false;
      }
    } else if (origin_mode_ != "first_valid_fix" && origin_mode_ != "service") {
      ROS_FATAL("~origin_mode must be 'first_valid_fix', 'parameter' or 'service', got '%s'",
                origin_mode_.c_str());
      return false;
    }

    // 读取输入话题并开始订阅。launch 参数 gnss_topic 会覆盖 YAML 中的同名值。
    std::string gnss_topic;
    private_nh_.param<std::string>("gnss_topic", gnss_topic, "/handsfree/rtk/gnss");
    gnss_sub_ = nh_.subscribe(gnss_topic, 100, &GeoCoordinateLayer::FixCallback, this);
    // raw NMEA 在 rtk_fixed_only 与 gga_min 模式中作为 GGA 质量判据；其他模式订阅它
    // 仅用于诊断，缺少该话题不会影响 any/not_no_fix/rtk_class 模式。
    if (!nmea_raw_topic_.empty()) {
      nmea_sub_ = nh_.subscribe(nmea_raw_topic_, 100, &GeoCoordinateLayer::RawNmeaCallback, this);
    }
    reset_origin_service_ = private_nh_.advertiseService(
        "set_origin_from_latest", &GeoCoordinateLayer::SetOriginFromLatest, this);

    if (!has_origin_) {
      PublishStatus("waiting for acceptable GNSS on " + gnss_topic +
                    ", origin_mode=" + origin_mode_ + ", quality_mode=" + quality_mode_);
    }
    ROS_INFO("geo_coordinate_layer: GNSS=%s, raw NMEA=%s, ENU frame=%s, origin mode=%s, "
             "quality mode=%s",
             gnss_topic.c_str(), nmea_raw_topic_.c_str(), enu_frame_id_.c_str(),
             origin_mode_.c_str(), quality_mode_.c_str());
    return true;
  }

 private:
  // 判断最近一次 GGA 质量是否仍可与当前 GNSS 数据对应。
  // std_msgs/String 没有 header，因此使用收到原始 NMEA 时的 ROS 时间做新鲜度判断。
  bool HasFreshGgaQuality() const {
    if (latest_gga_quality_ < 0 || latest_gga_quality_stamp_.isZero()) {
      return false;
    }
    return std::fabs((ros::Time::now() - latest_gga_quality_stamp_).toSec()) <=
           nmea_quality_timeout_sec_;
  }

  // GNSS 有效性与质量筛选。
  // quality_mode=any：只检查经纬度合法；not_no_fix：拒绝明确无定位；
  // rtk_class：要求厂商驱动映射到 RTK 类状态；rtk_fixed_only：要求新鲜 GGA=4；
  // gga_min：要求新鲜 GGA 质量不低于 ~min_gga_quality（默认 1，即接受单点定位）。
  bool IsValidFix(const sensor_msgs::NavSatFix& fix, std::string* reason) const {
    if (!std::isfinite(fix.latitude) || !std::isfinite(fix.longitude)) {
      *reason = "latitude/longitude is not finite";
      return false;
    }
    if (fix.latitude < -90.0 || fix.latitude > 90.0 ||
        fix.longitude < -180.0 || fix.longitude > 180.0) {
      *reason = "latitude/longitude is outside geographic bounds";
      return false;
    }
    if ((require_fix_status_ || quality_mode_ == "not_no_fix") &&
        fix.status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
      *reason = "GNSS status reports no fix";
      return false;
    }

    // UM982 文档中的 ROS 驱动将 RTK Fixed/Float 都映射为 STATUS_GBAS_FIX。
    // 因而 rtk_class 只能表示“RTK 类”，不能区分 Fixed 与 Float。
    if (quality_mode_ == "rtk_class" &&
        fix.status.status != sensor_msgs::NavSatStatus::STATUS_GBAS_FIX) {
      *reason = "quality_mode=rtk_class requires NavSatStatus::STATUS_GBAS_FIX";
      return false;
    }
    if (quality_mode_ == "rtk_fixed_only") {
      if (!HasFreshGgaQuality()) {
        *reason = "quality_mode=rtk_fixed_only is waiting for fresh GGA quality on " +
                  nmea_raw_topic_;
        return false;
      }
      if (latest_gga_quality_ != 4) {
        std::ostringstream stream;
        stream << "quality_mode=rtk_fixed_only requires GGA quality 4, got "
               << latest_gga_quality_;
        *reason = stream.str();
        return false;
      }
    }

    // NavSatFix 协方差以 ENU 米平方表示。开启阈值后，要求东、北两个对角项均已知
    // 且不超过上限；这样比旧版本只检查 [0]（东向）更适合实车质量门控。
    if (max_position_covariance_ >= 0.0) {
      if (fix.position_covariance_type == sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
        *reason = "position covariance is unknown while covariance filtering is enabled";
        return false;
      }
      const double horizontal_covariance =
          std::max(fix.position_covariance[0], fix.position_covariance[4]);
      if (!std::isfinite(horizontal_covariance) ||
          horizontal_covariance > max_position_covariance_) {
        std::ostringstream stream;
        stream << "maximum horizontal position covariance " << horizontal_covariance
               << " exceeds " << max_position_covariance_;
        *reason = stream.str();
        return false;
      }
    }
    // gga_min：按可配置门限接受单点/差分/RTK，用于室外轨迹采集阶段。
    if (quality_mode_ == "gga_min") {
      if (!HasFreshGgaQuality()) {
        *reason = "quality_mode=gga_min is waiting for fresh GGA quality on " + nmea_raw_topic_;
        return false;
      }
      if (latest_gga_quality_ < min_gga_quality_) {
        std::ostringstream stream;
        stream << "quality_mode=gga_min requires GGA quality >= " << min_gga_quality_
               << ", got " << latest_gga_quality_;
        *reason = stream.str();
        return false;
      }
    }

    return true;
  }

  // 保存一个 ENU 地理原点，并发布其 WGS-84 经纬高表示。
  // 该原点在 geo_enu 坐标系中永远是 (0, 0, 0)。
  bool SetOrigin(double latitude, double longitude, double altitude, const std::string& source) {
    if (!std::isfinite(latitude) || !std::isfinite(longitude) || latitude < -90.0 ||
        latitude > 90.0 || longitude < -180.0 || longitude > 180.0) {
      ROS_FATAL("The requested ENU origin is invalid.");
      return false;
    }
    origin_latitude_ = latitude;
    origin_longitude_ = longitude;
    origin_altitude_ = std::isfinite(altitude) ? altitude : 0.0;
    origin_ecef_ = GeodeticToEcef(origin_latitude_, origin_longitude_, origin_altitude_);
    has_origin_ = true;

    // /amap_nav/enu_origin 的作用是记录 ENU 原点在地球上的位置。
    // 这里的 status 是用于表示“存在原点”的固定占位值，不是输入 RTK 的真实质量。
    sensor_msgs::NavSatFix origin;
    origin.header.stamp = ros::Time::now();
    origin.header.frame_id = "wgs84";
    origin.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
    origin.latitude = origin_latitude_;
    origin.longitude = origin_longitude_;
    origin.altitude = origin_altitude_;
    origin_pub_.publish(origin);

    std::ostringstream stream;
    stream << std::fixed << std::setprecision(9)
           << "ENU origin set from " << source << ": lat=" << origin_latitude_
           << ", lon=" << origin_longitude_ << std::setprecision(3)
           << ", alt=" << origin_altitude_;
    PublishStatus(stream.str());
    ROS_WARN("%s. Downstream ENU consumers must be restarted if this origin changes.",
             stream.str().c_str());
    return true;
  }

  // 发布便于人工查看的状态文字，例如“等待 GNSS”或“原点已设置”。
  void PublishStatus(const std::string& text) const {
    std_msgs::String status;
    status.data = text;
    status_pub_.publish(status);
  }

  // 复制原始 NavSatFix 的时间戳、状态和协方差，仅替换经纬度为 GCJ-02。
  void PublishGcj02(const sensor_msgs::NavSatFix& fix) const {
    if (!publish_gcj02_) {
      return;
    }
    sensor_msgs::NavSatFix gcj02 = fix;
    gcj02.header.frame_id = "gcj02";
    Wgs84ToGcj02(fix.latitude, fix.longitude, &gcj02.latitude, &gcj02.longitude);
    gcj02_pub_.publish(gcj02);
  }

  // 缓存最近收到的 GGA 定位质量。该质量只在 rtk_fixed_only 模式中决定是否接收 GNSS。
  void RawNmeaCallback(const std_msgs::StringConstPtr& raw_nmea) {
    int quality = -1;
    if (!ParseLatestGgaQuality(raw_nmea->data, &quality)) {
      return;
    }
    latest_gga_quality_ = quality;
    latest_gga_quality_stamp_ = ros::Time::now();
    ROS_DEBUG_THROTTLE(5.0, "geo_coordinate_layer: latest GGA quality=%d", quality);
  }

  // 每收到一条 GNSS 数据就执行一次：筛选 -> 缓存 -> 必要时设原点 ->
  // 计算 ENU 米制坐标 -> 发布 ENU 与 GCJ-02。
  void FixCallback(const sensor_msgs::NavSatFixConstPtr& fix) {
    std::string reason;
    if (!IsValidFix(*fix, &reason)) {
      ROS_WARN_THROTTLE(5.0, "geo_coordinate_layer: ignored GNSS fix: %s", reason.c_str());
      return;
    }
    latest_fix_.reset(new sensor_msgs::NavSatFix(*fix));

    // first_valid_fix 模式下，第一条通过质量策略的 GNSS 数据定义局部原点。
    if (!has_origin_ && origin_mode_ == "first_valid_fix") {
      SetOrigin(fix->latitude, fix->longitude,
                std::isfinite(fix->altitude) ? fix->altitude : 0.0,
                "first valid GNSS fix");
    }

    // service 模式尚未由操作者锁定原点时，不发布没有参考系的 ENU；
    // 但仍可发布地图侧需要的 GCJ-02 经纬度。
    if (!has_origin_) {
      ROS_WARN_THROTTLE(5.0,
                        "geo_coordinate_layer: ENU is waiting for an origin; call "
                        "~set_origin_from_latest after confirming GNSS quality");
      PublishGcj02(*fix);
      return;
    }

    // 严格 WGS-84 局部 ENU：经纬高 -> ECEF -> 以原点旋转到 East-North-Up。
    // 这避免旧球面近似在校园长距离北向运动时产生的尺度误差。
    const EcefPoint point_ecef = GeodeticToEcef(
        fix->latitude, fix->longitude,
        std::isfinite(fix->altitude) ? fix->altitude : origin_altitude_);
    double east = 0.0;
    double north = 0.0;
    double up = 0.0;
    EcefToEnu(point_ecef, origin_ecef_, origin_latitude_, origin_longitude_,
              &east, &north, &up);

    // 发布局部位置。该节点没有航向输入，因此单位四元数只是“无旋转”的占位值，
    // 不应把它当作机器人朝向或与 /lio/odom 的姿态直接混用。
    geometry_msgs::PoseStamped pose;
    pose.header = fix->header;
    pose.header.frame_id = enu_frame_id_;
    pose.pose.position.x = east;
    pose.pose.position.y = north;
    pose.pose.position.z = up;
    pose.pose.orientation.w = 1.0;
    enu_pub_.publish(pose);
    PublishGcj02(*fix);
  }

  // 服务回调：将最近一次通过基础检查的 GNSS 数据设为新 ENU 原点。
  // 这会改变后续所有 ENU 坐标，调用前应停止下游控制器。
  bool SetOriginFromLatest(std_srvs::Trigger::Request&,
                           std_srvs::Trigger::Response& response) {
    if (!latest_fix_) {
      response.success = false;
      response.message = "No valid GNSS fix has arrived yet.";
      return true;
    }
    const double altitude = std::isfinite(latest_fix_->altitude) ? latest_fix_->altitude : 0.0;
    response.success = SetOrigin(latest_fix_->latitude, latest_fix_->longitude,
                                 altitude, "service");
    response.message = response.success ? "ENU origin reset from latest valid GNSS fix."
                                        : "Latest GNSS fix could not be used as an origin.";
    return true;
  }

  // ROS 通信对象：订阅 GNSS/NMEA、发布四类输出、提供重设原点服务。
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber gnss_sub_;
  ros::Subscriber nmea_sub_;
  ros::Publisher enu_pub_;
  ros::Publisher gcj02_pub_;
  ros::Publisher origin_pub_;
  ros::Publisher status_pub_;
  ros::ServiceServer reset_origin_service_;

  // 从参数服务器读取的运行配置，以及当前 ENU 原点与最新 GNSS 缓存。
  std::string enu_frame_id_;
  std::string origin_mode_;
  std::string quality_mode_;
  std::string nmea_raw_topic_;
  bool require_fix_status_ = false;
  bool publish_gcj02_ = true;
  int min_gga_quality_ = 1;
  double max_position_covariance_ = -1.0;
  double nmea_quality_timeout_sec_ = 1.0;
  bool has_origin_ = false;
  double origin_latitude_ = 0.0;
  double origin_longitude_ = 0.0;
  double origin_altitude_ = 0.0;
  EcefPoint origin_ecef_;
  int latest_gga_quality_ = -1;
  ros::Time latest_gga_quality_stamp_;
  std::unique_ptr<sensor_msgs::NavSatFix> latest_fix_;
};

}  // namespace amap_outdoor_indoor_nav

int main(int argc, char** argv) {
  // ROS 进程入口：初始化节点、完成一次性配置，然后持续等待 GNSS 回调。
  ros::init(argc, argv, "geo_coordinate_layer");
  amap_outdoor_indoor_nav::GeoCoordinateLayer node;
  if (!node.Initialise()) {
    return 1;
  }
  ros::spin();
  return 0;
}
