import java.util.Comparator;

public class DStarComparator implements Comparator<double[]> {
    @Override
    public int compare(double[] a, double[] b) {
        // 假设a和b的长度至少为4，并且第3、4个元素（MATLAB中的a(3),a(4)）为关键字
        // 在Java中a(3)对应a[2], a(4)对应a[3]
        
        // 比较第3个元素（索引2）
        if (a[2] < b[2] || (a[2] == b[2] && a[3] < b[3])) {
            return -1;
        } else if (a[2] > b[2] || (a[2] == b[2] && a[3] > b[3])) {
            return 1;
        } else {
            return 0;
        }
    }
}
