/*
 * JavaDriverUm98xLinux.java — Build & Run Notes
 * -------------------------------------------------
 * 本程序为“POSIX 串口方案”（stty + 文件流），适用于 Linux / macOS（/dev/tty*）。
 *
 * 依赖/环境
 * - JDK 8+：`sudo apt install -y openjdk-11-jdk`
 * 
 * 编译（保存文件名与类名一致：JavaDriverUm98xLinux.java）
 *   javac JavaDriverUm98xLinux.java
 * （可选）打包 JAR：
 *   echo "Main-Class: JavaDriverUm98xLinux" > MANIFEST.MF
 *   jar cfm um98x.jar MANIFEST.MF JavaDriverUm98xLinux.class
 *
 * 运行（Linux/macOS）
 * - 列出候选串口：
 *     java JavaDriverUm98xLinux --list
 * - 读取（按需选择权限；若无权限再加 sudo）：
 *     java JavaDriverUm98xLinux --port /dev/ttyUSB0 --baud 115200 --timeout 100
 * - 仅打印解析后的结构化日志（不打印原始 NMEA）：
 *     java JavaDriverUm98xLinux --port /dev/ttyUSB0 --no-raw
 *
 * 参数速查
 *   --list                 列出候选串口
 *   --port <dev>          串口设备（默认 /dev/ttyUSB0）
 *   --baud <rate>         波特率（默认 115200）
 *   --timeout <ms>        读超时（0.1s 步进；默认 100）
 *   --no-raw              不打印原始 $GGA/$RMC/$THS 行
 *   -h / --help           显示帮助
 */

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.time.*;
import java.time.format.DateTimeFormatter;
import java.util.*;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class JavaDriverUm98xLinux {

    // ===== CLI 参数 =====
    private static class Args {
        String port = "/dev/ttyUSB0";
        int baud = 115200;
        int timeoutMs = 100;     // 读超时 (近似, 0.1s 步进)
        boolean noRaw = false;
        boolean list = false;
    }

    private static Args parseArgs(String[] args) {
        Args a = new Args();
        for (int i = 0; i < args.length; i++) {
            String k = args[i];
            switch (k) {
                case "--port":    a.port = args[++i]; break;
                case "--baud":    a.baud = Integer.parseInt(args[++i]); break;
                case "--timeout": a.timeoutMs = Integer.parseInt(args[++i]); break;
                case "--no-raw":  a.noRaw = true; break;
                case "--list":    a.list = true; break;
                case "-h":
                case "--help":    printHelpAndExit();
                default:
                    if (k.startsWith("-")) { System.err.println("Unknown arg: " + k); printHelpAndExit(); }
            }
        }
        return a;
    }

    private static void printHelpAndExit() {
        System.out.println("NMEA RTK serial reader (POSIX, no dependencies)");
        System.out.println("Usage:");
        System.out.println("  java JavaDriverUm98xLinux [--list] [--port <dev>] [--baud <rate>] [--timeout <ms>] [--no-raw]");
        System.out.println("Examples:");
        System.out.println("  java JavaDriverUm98xLinux --list");
        System.out.println("  sudo java JavaDriverUm98xLinux --port /dev/ttyUSB0 --baud 115200 --timeout 100");
        System.exit(0);
    }

    private static void listPorts() {
        List<String> patterns = Arrays.asList("/dev/ttyUSB*", "/dev/ttyACM*", "/dev/cu.*"); // Linux + macOS
        System.out.println("Candidate serial devices:");
        for (String pat : patterns) {
            try (DirectoryStream<Path> ds = Files.newDirectoryStream(Paths.get("/"), path -> {
                // 手工 glob：简易匹配
                return path.toString().startsWith("/dev/");
            })) {
                // 改用简单的 Files.newDirectoryStream("/dev", glob) 更直接
            } catch (IOException ignored) {}
        }
        try {
            Files.newDirectoryStream(Paths.get("/dev"), "ttyUSB*").forEach(p -> System.out.println("  " + p));
        } catch (IOException ignored) {}
        try {
            Files.newDirectoryStream(Paths.get("/dev"), "ttyACM*").forEach(p -> System.out.println("  " + p));
        } catch (IOException ignored) {}
        try {
            // macOS
            Files.newDirectoryStream(Paths.get("/dev"), "cu.*").forEach(p -> System.out.println("  " + p));
        } catch (IOException ignored) {}
    }

    // ===== 简易日志 =====
    private static void log(String level, String msg) {
        String ts = DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss.SSS")
                .withZone(ZoneOffset.UTC).format(Instant.now());
        System.out.printf("[%s][%s] %s%n", ts, level, msg);
    }

    // ===== 串口（POSIX）封装：用 stty 配置，FileInputStream 读取 =====
    private final String portName;
    private final int baudrate;
    private final int timeoutMs;
    private final boolean printRaw;

    private final AtomicBoolean stop = new AtomicBoolean(false);
    private Thread readThread;
    private FileInputStream fis;
    private FileDescriptor fd;

    public JavaDriverUm98xLinux(String portName, int baudrate, int timeoutMs, boolean printRaw) {
        this.portName = portName;
        this.baudrate = baudrate;
        this.timeoutMs = timeoutMs;
        this.printRaw = printRaw;
    }

    public void run() {
        log("INFO", String.format("Opening %s @ %d (POSIX, stty)...", portName, baudrate));
        while (!stop.get()) {
            try {
                if (!configurePortPOSIX()) {
                    log("ERROR", "stty configure failed. Retry in 1s...");
                    sleepMs(1000);
                    continue;
                }
                if (!openStream()) {
                    log("ERROR", "open stream failed. Retry in 1s...");
                    sleepMs(1000);
                    continue;
                }
                log("INFO", "Serial opened: " + portName);
                startReader();
                while (!stop.get() && readThread.isAlive()) {
                    sleepMs(100);
                }
            } finally {
                closeStream();
            }
            if (!stop.get()) {
                log("INFO", "Reconnecting in 1s...");
                sleepMs(1000);
            }
        }
        log("INFO", "Reader stopped.");
    }

    public void stop() {
        log("INFO", "Stopping NMEA reader...");
        stop.set(true);
        if (readThread != null && readThread.isAlive()) {
            try { readThread.join(1000); } catch (InterruptedException ignored) {}
        }
        closeStream();
        log("INFO", "Stopped.");
    }

    private boolean configurePortPOSIX() {
        int timeDeci = Math.max(0, Math.min(255, (int)Math.round(timeoutMs / 100.0)));
        List<String> cmd = Arrays.asList(
            "sh", "-c",
            String.format(
                // 关键：min 1（至少读到1字节才返回）；time 仍用 0.1s 步进的近似超时
                "stty -F %s %d cs8 -parenb -cstopb -ixon -ixoff -crtscts -icanon -echo -isig -opost time %d min 1",
                shellEscape(portName), baudrate, timeDeci
            )
        );
        return runAndCheck(cmd);
    }

    private String shellEscape(String s) {
        // 简易防空格：若包含空格则加引号
        if (s.contains(" ")) return "'" + s.replace("'", "'\\''") + "'";
        return s;
    }

    private boolean runAndCheck(List<String> cmd) {
        try {
            Process p = new ProcessBuilder(cmd).redirectErrorStream(true).start();
            ByteArrayOutputStream bos = new ByteArrayOutputStream();
            try (InputStream is = p.getInputStream()) {
                byte[] buf = new byte[1024];
                int n;
                while ((n = is.read(buf)) != -1) bos.write(buf, 0, n);
            }
            int code = p.waitFor();
            if (code != 0) {
                log("ERROR", "Command failed: " + String.join(" ", cmd));
                log("ERROR", "Output: " + new String(bos.toByteArray(), StandardCharsets.UTF_8));
            }
            return code == 0;
        } catch (Exception e) {
            log("ERROR", "Run stty error: " + e.getMessage());
            return false;
        }
    }

    private boolean openStream() {
        try {
            fis = new FileInputStream(new File(portName));
            fd = fis.getFD();
            return true;
        } catch (Exception e) {
            log("ERROR", "openStream: " + e.getMessage());
            return false;
        }
    }

    private void closeStream() {
        try { if (fis != null) fis.close(); } catch (Exception ignored) {}
        fis = null; fd = null;
    }

    private void startReader() {
        readThread = new Thread(() -> {
            log("INFO", "NMEA reading thread started.");
            try (BufferedReader br = new BufferedReader(new InputStreamReader(fis, StandardCharsets.UTF_8))) {
                String line;
                while (!stop.get() && (line = br.readLine()) != null) {
                    line = line.trim();
                    if (line.isEmpty()) continue;

                    if (!printRaw) {
                        // 不打印任何原始行
                    } else {
                        if (line.startsWith("$GNGGA") || line.startsWith("$GPGGA")
                                || line.startsWith("$GNRMC") || line.startsWith("$GPRMC")
                                || line.startsWith("$GNTHS") || line.startsWith("$GPTHS")) {
                            log("INFO", line);
                        }
                    }

                    NMEAParsed parsed = parseNmea(line);
                    if (parsed == null) continue;

                    switch (parsed.type) {
                        case GGA:
                            int fixQ = parsed.fixQuality != null ? parsed.fixQuality : -1;
                            FixInfo info = FIX_TYPE_MAPPING.getOrDefault(fixQ, FIX_UNKNOWN);
                            int sats = parsed.numSats != null ? parsed.numSats : -1;
                            double hdop = parsed.hdop != null ? parsed.hdop : -1.0;
                            double lat = parsed.latitude != null ? parsed.latitude : Double.NaN;
                            double lon = parsed.longitude != null ? parsed.longitude : Double.NaN;
                            double alt = parsed.altitude != null ? parsed.altitude : 0.0;
                            log("INFO", String.format(Locale.US,
                                    "GGA: fix=%d (%s), sats=%d, hdop=%.2f, lat=%.8f, lon=%.8f, alt=%.3f m",
                                    fixQ, info.label, sats, hdop, lat, lon, alt));
                            break;
                        case RMC:
                            char st = parsed.status != null ? parsed.status : 'V';
                            String mode = GNRMC_MODE_MAPPING.getOrDefault(st, "Unknown");
                            double spd = parsed.speedMps != null ? parsed.speedMps : 0.0;
                            double spdKn = parsed.rawSpeedKnots != null ? parsed.rawSpeedKnots : 0.0;
                            double cog = parsed.cogDeg != null ? parsed.cogDeg : 0.0;
                            log("INFO", String.format(Locale.US,
                                    "RMC: status=%s, speed=%.3f m/s (%.3f kn), COG=%.2f°",
                                    mode, spd, spdKn, cog));
                            break;
                        case THS:
                            if (parsed.heading != null) {
                                log("INFO", String.format(Locale.US,
                                        "THS: heading=%.3f° (%s)",
                                        parsed.heading, parsed.thsValid ? "valid" : "invalid"));
                            }
                            break;
                    }
                }
            } catch (Exception e) {
                if (!stop.get()) log("ERROR", "read error: " + e.getMessage());
            }
        }, "nmea_reader");
        readThread.setDaemon(true);
        readThread.start();
    }

    // ===== 状态映射，与 Python 版一致 =====
    private static final class FixInfo {
        final String label, statusCode; final double covM;
        FixInfo(String a, String b, double c) { label=a; statusCode=b; covM=c; }
    }
    private static final Map<Integer, FixInfo> FIX_TYPE_MAPPING = new HashMap<>();
    private static final FixInfo FIX_UNKNOWN = new FixInfo("Unknown", "NO_FIX", 10000.0);
    private static final Map<Character, String> GNRMC_MODE_MAPPING = new HashMap<>();
    static {
        FIX_TYPE_MAPPING.put(0, new FixInfo("Invalid Fix", "NO_FIX", 10000.0));
        FIX_TYPE_MAPPING.put(1, new FixInfo("GPS Fix (SPS)", "FIX", 1.0));
        FIX_TYPE_MAPPING.put(2, new FixInfo("DGPS Fix", "SBAS_FIX", 0.5));
        FIX_TYPE_MAPPING.put(3, new FixInfo("PPS Fix", "NO_FIX", 10000.0));
        FIX_TYPE_MAPPING.put(4, new FixInfo("RTK Fixed", "GBAS_FIX", 0.01));
        FIX_TYPE_MAPPING.put(5, new FixInfo("RTK Float", "GBAS_FIX", 0.1));
        FIX_TYPE_MAPPING.put(6, new FixInfo("Estimated", "FIX", 5.0));
        FIX_TYPE_MAPPING.put(7, new FixInfo("Manual", "GBAS_FIX", 0.01));
        FIX_TYPE_MAPPING.put(8, new FixInfo("Simulation", "FIX", 10000.0));

        GNRMC_MODE_MAPPING.put('A', "Autonomous Mode");
        GNRMC_MODE_MAPPING.put('D', "Differential Mode");
        GNRMC_MODE_MAPPING.put('E', "INS Mode");
        GNRMC_MODE_MAPPING.put('F', "RTK Float");
        GNRMC_MODE_MAPPING.put('M', "Manual Input Mode");
        GNRMC_MODE_MAPPING.put('N', "No Fix");
        GNRMC_MODE_MAPPING.put('P', "Precision Mode");
        GNRMC_MODE_MAPPING.put('R', "RTK Fixed");
        GNRMC_MODE_MAPPING.put('S', "Simulator Mode");
        GNRMC_MODE_MAPPING.put('V', "Invalid Mode");
    }

    // ===== NMEA 解析（GGA/RMC/THS） =====
    private enum Type { GGA, RMC, THS }
    private static final Pattern DMS_PATTERN = Pattern.compile("^(\\d+)(\\d\\d\\.\\d+)$");

    private static class NMEAParsed {
        Type type;
        Instant timeUtc;

        // GGA
        Double latitude, longitude, altitude, hdop;
        Integer fixQuality, numSats;

        // RMC
        Character status;
        Double speedMps, rawSpeedKnots, cogDeg;
        String dateDDMMYY;

        // THS
        Double heading;
        boolean thsValid;
    }

    private static NMEAParsed parseNmea(String s) {
        try {
            if (s.startsWith("$GNGGA") || s.startsWith("$GPGGA")) {
                String[] p = s.split(",", -1);
                if (p.length < 10) return null;
                String timeUtc = p[1];
                Double lat = dmsToDecimal(p[2], p[3]);
                Double lon = dmsToDecimal(p[4], p[5]);
                Integer fixQ = toIntOr(p[6], 0);
                Integer numSats = toIntOr(p[7], 0);
                Double hdop = toDoubleOr(p[8], 99.9);
                Double alt = toDoubleOr(p[9], 0.0);

                NMEAParsed n = new NMEAParsed();
                n.type = Type.GGA;
                n.timeUtc = nmeaTimeToInstant(timeUtc);
                n.latitude = lat; n.longitude = lon; n.altitude = alt;
                n.fixQuality = fixQ; n.numSats = numSats; n.hdop = hdop;
                return n;
            }
            if (s.startsWith("$GNRMC") || s.startsWith("$GPRMC")) {
                String[] p = s.split(",", -1);
                if (p.length < 12) return null;
                String timeUtc = p[1];
                Character status = p[2].isEmpty() ? null : p[2].charAt(0);
                Double lat = dmsToDecimal(p[3], p[4]);
                Double lon = dmsToDecimal(p[5], p[6]);
                Double spKn = toDoubleOr(p[7], 0.0);
                Double cog = toDoubleOr(p[8], 0.0);
                String date = p[9];

                NMEAParsed n = new NMEAParsed();
                n.type = Type.RMC;
                n.timeUtc = nmeaTimeToInstant(timeUtc);
                n.status = status;
                n.latitude = lat; n.longitude = lon;
                n.rawSpeedKnots = spKn;
                n.speedMps = spKn * 0.514444;
                n.cogDeg = cog;
                n.dateDDMMYY = date;
                return n;
            }
            if (s.startsWith("$GNTHS") || s.startsWith("$GPTHS")) {
                String[] p = s.split(",", -1);
                if (p.length < 3) return null;
                Double heading = toDoubleOr(p[1], null);
                boolean valid = !p[2].isEmpty() && p[2].charAt(0) == 'A';
                NMEAParsed n = new NMEAParsed();
                n.type = Type.THS;
                n.heading = heading;
                n.thsValid = valid;
                return n;
            }
            return null;
        } catch (Exception e) {
            return null;
        }
    }

    private static Instant nmeaTimeToInstant(String hhmmss) {
        if (hhmmss == null || hhmmss.isEmpty()) return Instant.now();
        try {
            int hh = Integer.parseInt(hhmmss.substring(0, 2));
            int mm = Integer.parseInt(hhmmss.substring(2, 4));
            double sFrac = (hhmmss.length() > 4) ? Double.parseDouble(hhmmss.substring(4)) : 0.0;
            int ss = (int) sFrac;
            int micros = (int) Math.round((sFrac - ss) * 1_000_000);
            LocalDate today = LocalDate.now(ZoneOffset.UTC);
            LocalDateTime ldt = LocalDateTime.of(today.getYear(), today.getMonthValue(), today.getDayOfMonth(),
                    hh, mm, ss, micros * 1000);
            return ldt.toInstant(ZoneOffset.UTC);
        } catch (Exception e) {
            return Instant.now();
        }
    }

    private static Double dmsToDecimal(String dms, String dir) {
        try {
            if (dms == null || dms.isEmpty()) return null;
            Matcher m = DMS_PATTERN.matcher(dms);
            if (!m.find()) return null;
            double deg = Double.parseDouble(m.group(1));
            double minutes = Double.parseDouble(m.group(2));
            double dec = deg + minutes / 60.0;
            if ("S".equals(dir) || "W".equals(dir)) dec = -dec;
            return dec;
        } catch (Exception e) { return null; }
    }

    private static Integer toIntOr(String s, int def) {
        try { return (s == null || s.isEmpty()) ? def : Integer.parseInt(s); }
        catch (Exception e) { return def; }
    }

    private static Double toDoubleOr(String s, Double def) {
        try { return (s == null || s.isEmpty()) ? def : Double.parseDouble(s); }
        catch (Exception e) { return def; }
    }

    private static void sleepMs(long ms) {
        try { Thread.sleep(ms); } catch (InterruptedException ignored) {}
    }

    // ===== main =====
    public static void main(String[] argv) {
        Args args = parseArgs(argv);
        if (args.list) { listPorts(); return; }

        JavaDriverUm98xLinux reader = new JavaDriverUm98xLinux(args.port, args.baud, args.timeoutMs, !args.noRaw);
        Runtime.getRuntime().addShutdownHook(new Thread(reader::stop, "shutdown"));
        reader.run();
    }
}
