package frc.robot.util;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.File;
import java.io.IOException;

/**
 * PathPlanner NavGrid dosyasını okuyarak engel kontrolü yapar.
 * 
 * <p>
 * Teleop sırasında robotun engellere girmesini önlemek için kullanılır.
 * NavGrid dosyası PathPlanner tarafından oluşturulur ve sahadaki
 * engel alanlarını tanımlar.
 * </p>
 * 
 * @author FRC Team
 */
public class NavGridUtil {

    private static final String NAVGRID_PATH = "pathplanner/navgrid.json";

    private static boolean[][] grid;
    private static double nodeSizeMeters;
    private static double fieldWidth;
    private static double fieldHeight;
    private static boolean loaded = false;

    static {
        loadNavGrid();
    }

    /**
     * NavGrid dosyasını yükler.
     */
    private static void loadNavGrid() {
        try {
            File navGridFile = new File(Filesystem.getDeployDirectory(), NAVGRID_PATH);
            if (!navGridFile.exists()) {
                System.out.println("[NavGrid] navgrid.json bulunamadı, engel kontrolü devre dışı");
                return;
            }

            ObjectMapper mapper = new ObjectMapper();
            JsonNode root = mapper.readTree(navGridFile);

            // Saha boyutları
            fieldWidth = root.get("field_size").get("x").asDouble();
            fieldHeight = root.get("field_size").get("y").asDouble();
            nodeSizeMeters = root.get("nodeSizeMeters").asDouble();

            // Grid'i oku
            JsonNode gridNode = root.get("grid");
            int rows = gridNode.size();
            int cols = gridNode.get(0).size();
            grid = new boolean[rows][cols];

            for (int y = 0; y < rows; y++) {
                JsonNode row = gridNode.get(y);
                for (int x = 0; x < cols; x++) {
                    // true = engel (geçilemez), false = boş alan
                    grid[y][x] = row.get(x).asBoolean();
                }
            }

            loaded = true;
            System.out.println("[NavGrid] Yüklendi: " + cols + "x" + rows +
                    " grid, nodeSize=" + nodeSizeMeters + "m");
        } catch (IOException e) {
            System.out.println("[NavGrid] Yükleme hatası: " + e.getMessage());
        }
    }

    /**
     * Verilen pozisyonun engel içinde olup olmadığını kontrol eder.
     * 
     * @param position Kontrol edilecek pozisyon
     * @return true = engel var (geçilemez), false = boş alan
     */
    public static boolean isBlocked(Translation2d position) {
        if (!loaded || grid == null) {
            return false; // NavGrid yüklenmemişse engel yok kabul et
        }

        // Pozisyonu grid indekslerine çevir
        int x = (int) (position.getX() / nodeSizeMeters);
        int y = (int) (position.getY() / nodeSizeMeters);

        // Sınır kontrolü
        if (y < 0 || y >= grid.length || x < 0 || x >= grid[0].length) {
            return true; // Saha dışı = engel
        }

        return grid[y][x];
    }

    /**
     * Verilen pozisyona hareket ederken engele çarpılıp çarpılmayacağını kontrol
     * eder.
     * Robotun mevcut pozisyonundan hedef pozisyona bir çizgi çekip ara noktaları
     * kontrol eder.
     * 
     * @param current           Mevcut pozisyon
     * @param target            Hedef pozisyon
     * @param robotRadiusMeters Robot yarıçapı (bumper dahil)
     * @return true = yolda engel var, false = yol açık
     */
    public static boolean isPathBlocked(Translation2d current, Translation2d target, double robotRadiusMeters) {
        if (!loaded || grid == null) {
            return false;
        }

        // İki nokta arasındaki mesafe
        double distance = current.getDistance(target);
        if (distance < 0.01) {
            return isBlocked(target);
        }

        // Her nodeSize kadar bir kontrol yap
        int checkPoints = (int) Math.ceil(distance / (nodeSizeMeters / 2));
        for (int i = 0; i <= checkPoints; i++) {
            double t = (double) i / checkPoints;
            double x = current.getX() + t * (target.getX() - current.getX());
            double y = current.getY() + t * (target.getY() - current.getY());

            // Robot yarıçapını da kontrol et (4 köşe)
            if (isBlocked(new Translation2d(x + robotRadiusMeters, y)) ||
                    isBlocked(new Translation2d(x - robotRadiusMeters, y)) ||
                    isBlocked(new Translation2d(x, y + robotRadiusMeters)) ||
                    isBlocked(new Translation2d(x, y - robotRadiusMeters))) {
                return true;
            }
        }

        return false;
    }

    /**
     * NavGrid'in yüklenip yüklenmediğini kontrol eder.
     */
    public static boolean isLoaded() {
        return loaded;
    }

    /**
     * Saha genişliğini döndürür (metre).
     */
    public static double getFieldWidth() {
        return fieldWidth;
    }

    /**
     * Saha yüksekliğini döndürür (metre).
     */
    public static double getFieldHeight() {
        return fieldHeight;
    }
}
