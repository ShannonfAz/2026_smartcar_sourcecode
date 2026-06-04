#include "byd_math.hpp"
float byd_math::cos(cv::Point2f A,cv::Point2f B,cv::Point2f C){
    cv::Point2f BA = A - B;
    cv::Point2f BC = C - B;
    if(BA == cv::Point2f(0,0) || BC == cv::Point2f(0,0)){
        std::cerr << "操你妈，传进来俩一样的点" << std::endl;
        return -1;
    }
    float _BA = cv::norm(BA);
    float _BC = cv::norm(BC);
    float dot = BA.dot(BC);
    return dot/(_BA*_BC);
}
/*    0
* 0.5 + -0.5
*     1
*/
float byd_math::angle(cv::Point2f A,cv::Point2f B){
    return std::atan2(static_cast<float>(A.x-B.x),static_cast<float>(A.y-B.y))/3.14159265358979323846f;
}
float byd_math::AOBangle(const cv::Point2f& A, const cv::Point2f& B) {
    double cross = A.x * B.y - A.y * B.x;   
    double dot   = A.x * B.x + A.y * B.y;   
    return std::atan2(cross, dot)/3.14159265358979323846f;
}
float byd_math::AB_to_C(cv::Point2f A,cv::Point2f B,cv::Point2f C){
    cv::Point2f AB = B - A;
    cv::Point2f AC = C - A;
    float cross = abs(AB.x * AC.y - AB.y * AC.x);
    float normAB = cv::norm(AB);
    if(normAB < 0.01){
        // std::cerr << "操你妈，传进来俩一样的点" << std::endl;
        return cv::norm(AC);
    }
    return cross / normAB;
}
float byd_math::k(cv::Point2f A,cv::Point2f B){
    if(abs(B.y - A.y) < 0.1){
        return 114514.0;
    }else{
        return (B.x - A.x)/(B.y-A.y);
    }
}
float byd_math::b(cv::Point2f A,float k){
    return A.x - k*A.y;
}
float byd_math::buxian(float y,float k,float b){
    return k*y +b;
}
cv::Point byd_math::IPMPoint(std::vector<std::vector<float>> H,cv::Point input){
    cv::Point temp;
    temp.x = (input.x * H.at(0).at(0) + input.y * H.at(0).at(1) + H.at(0).at(2))
    /(input.x * H.at(2).at(0) + input.y * H.at(2).at(1) + H.at(2).at(2));
    temp.y = (input.x * H.at(1).at(0) + input.y * H.at(1).at(1) + H.at(1).at(2))
    /(input.x * H.at(2).at(0) + input.y * H.at(2).at(1) + H.at(2).at(2));
    return temp;
}
std::vector<int> byd_math::json_output_place(const std::vector<int>& a) {
    int n = a.size();
    if (n == 0) return std::vector<int>(64, 0);
    if (n == 64) return a;

    // 判定函数：在最大间隔 d 下，最少需要多少点覆盖 [a[0], a.back()] 并包含所有 a
    auto min_points = [&](long long d) -> int {
        if (d < 0) return INT_MAX;
        if (d == 0) {
            for (int i = 1; i < n; ++i)
                if (a[i] != a[0]) return INT_MAX;
            return n;
        }
        long long cur = a[0];
        int cnt = 1;
        for (int i = 1; i < n; ++i) {
            if (a[i] > cur + d) {
                long long diff = a[i] - cur;
                long long k = (diff - 1) / d; // 需要插入的点数
                cnt += k;
                cur += k * d;
            }
            cur = a[i];
            cnt++;
        }
        return cnt;
    };

    long long low = 0, high = a.back() - a.front();
    while (low < high) {
        long long mid = (low + high) / 2;
        if (min_points(mid) <= 64)
            high = mid;
        else
            low = mid + 1;
    }
    long long d = low;

    // 特殊情况：所有元素相同
    if (d == 0) {
        return std::vector<int>(64, a[0]);
    }

    // 生成最少点序列（保证以 a[0] 开始，a.back() 结尾）
    std::vector<int> seq;
    seq.push_back(a[0]);
    long long cur = a[0];
    for (int i = 1; i < n; ++i) {
        while (a[i] > cur + d) {
            cur += d;
            seq.push_back((int)cur);
        }
        cur = a[i];
        seq.push_back(a[i]);
    }

    // 填充到恰好 64 个点
    while (seq.size() < 64) {
        // 寻找当前最大间隔的位置
        size_t best_idx = 0;
        int max_gap = 0;
        for (size_t i = 0; i + 1 < seq.size(); ++i) {
            int gap = seq[i + 1] - seq[i];
            if (gap > max_gap) {
                max_gap = gap;
                best_idx = i;
            }
        }
        // 在最大间隔的中点插入新点
        int mid_val = seq[best_idx] + max_gap / 2;
        seq.insert(seq.begin() + best_idx + 1, mid_val);
    }

    return seq;
}



















byd_math::kdtree::kdtree(int n){
    MAXN = n+1;
    s.resize(MAXN);
    d.resize(MAXN);
    lc.resize(MAXN);
    rc.resize(MAXN);
    L.resize(MAXN);
    R.resize(MAXN);
    U.resize(MAXN);
    D.resize(MAXN);
}
void byd_math::kdtree::maintain(int x) {
    L[x] = R[x] = s[x].x;
    D[x] = U[x] = s[x].y;
    if (lc[x])
        L[x] = std::min(L[x], L[lc[x]]), R[x] = std::max(R[x], R[lc[x]]),
        D[x] = std::min(D[x], D[lc[x]]), U[x] = std::max(U[x], U[lc[x]]);
    if (rc[x])
        L[x] = std::min(L[x], L[rc[x]]), R[x] = std::max(R[x], R[rc[x]]),
        D[x] = std::min(D[x], D[rc[x]]), U[x] = std::max(U[x], U[rc[x]]);
}
double byd_math::kdtree::dist(int a, int b) {
    return (s[a].x - s[b].x) * (s[a].x - s[b].x) +
            (s[a].y - s[b].y) * (s[a].y - s[b].y);
}
double byd_math::kdtree::f(int a, int b) {
    double ret = 0;
    if (L[b] > s[a].x) ret += (L[b] - s[a].x) * (L[b] - s[a].x);
    if (R[b] < s[a].x) ret += (s[a].x - R[b]) * (s[a].x - R[b]);
    if (D[b] > s[a].y) ret += (D[b] - s[a].y) * (D[b] - s[a].y);
    if (U[b] < s[a].y) ret += (s[a].y - U[b]) * (s[a].y - U[b]);
    return ret;
}
double byd_math::kdtree::f(const cv::Point& coord, int box_idx) const {
    double ret = 0.0;
    if (L[box_idx] > coord.x) ret += (L[box_idx] - coord.x) * (L[box_idx] - coord.x);
    if (R[box_idx] < coord.x) ret += (coord.x - R[box_idx]) * (coord.x - R[box_idx]);
    if (D[box_idx] > coord.y) ret += (D[box_idx] - coord.y) * (D[box_idx] - coord.y);
    if (U[box_idx] < coord.y) ret += (coord.y - U[box_idx]) * (coord.y - U[box_idx]);
    return ret;
}
void byd_math::kdtree::nearest_rec(const cv::Point& target, int l, int r,int& best_idx, double& best_dist) const {
    if (l > r) return;
    int mid = (l + r) >> 1;
    double d2 = (target.x - s[mid].x) * (target.x - s[mid].x) + (target.y - s[mid].y) * (target.y - s[mid].y);
    if (d2 < best_dist) {
        best_dist = d2;
        best_idx = mid;
    }
    if (l == r) return;
    double distl = f(target, lc[mid]);
    double distr = f(target, rc[mid]);
    if (distl < best_dist && distr < best_dist) {
        if (distl < distr) {
            nearest_rec(target, l, mid - 1, best_idx, best_dist);
            if (distr < best_dist)nearest_rec(target, mid + 1, r, best_idx, best_dist);
        } else {
            nearest_rec(target, mid + 1, r, best_idx, best_dist);
            if (distl < best_dist)nearest_rec(target, l, mid - 1, best_idx, best_dist);
        }
    } else {
        if (distl < best_dist)nearest_rec(target, l, mid - 1, best_idx, best_dist);
        if (distr < best_dist)nearest_rec(target, mid + 1, r, best_idx, best_dist);
    }
}
int byd_math::kdtree::build(int l, int r) {
    if (l > r) return 0;
    if (l == r) {
        maintain(l);
        return l;
    }

    int mid = (l + r) >> 1;
    double avx = 0, avy = 0, vax = 0, vay = 0;
    for (int i = l; i <= r; i++) avx += s[i].x, avy += s[i].y;
    avx /= (double)(r - l + 1);
    avy /= (double)(r - l + 1);

    for (int i = l; i <= r; i++)
        vax += (s[i].x - avx) * (s[i].x - avx),
                vay += (s[i].y - avy) * (s[i].y - avy);
    if (vax >= vay)
        d[mid] = 1, std::nth_element(s.begin() + l, s.begin() + mid, s.begin() + r + 1, [](cv::Point a, cv::Point b) { return a.x < b.x; });
    else
        d[mid] = 2, std::nth_element(s.begin() + l, s.begin() + mid, s.begin() + r + 1, [](cv::Point a, cv::Point b) { return a.y < b.y; });
    lc[mid] = build(l, mid - 1), rc[mid] = build(mid + 1, r);
    maintain(mid);
    return mid;
}
std::size_t byd_math::kdtree::nearest_index(const cv::Point& coord) const {
    int best_idx = -1;
    double best_dist = std::numeric_limits<double>::max();
    nearest_rec(coord, 1, MAXN, best_idx, best_dist);
    return static_cast<std::size_t>(best_idx);
}