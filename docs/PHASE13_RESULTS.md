# Full Benchmark Validation Report (Phase 13)

## 1. Dead Reckoning Drift Performance (60s GNSS Blackout)
**Official Benchmark Target**: ${{ < 10.0\% }}$ of total distance travelled during GNSS outage.
**Team Stretch Target**: ${{ < 5.0\% }}$ (Optimized via HD Map Heading Clamping).

| Session ID | Vehicle Category | Data Source / Platform | Outage Dist (m) | Final Error (m) | Drift % |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **S4** | Car | IO-VNBD (MEMS) | 498.70 | 35.83 | **7.19%** |
| **Vta28** | Car | IO-VNBD (MEMS) | 696.07 | 82.99 | **11.92%** |
| **Vtb11** | Car | IO-VNBD (MEMS) | 224.93 | 52.44 | **23.31%** |
| **Vw17** | Car | IO-VNBD (MEMS) | 169.99 | 22.03 | **12.96%** |
| **session1** | Two Wheeler | Bridge Synthetic | 231.32 | 729.28 | **315.27%** |

### Summary of Optimized Findings:
- **Best Case (Car)**: S4 at 7.19% drift using localized OSM constraints.
- **Physical Limitation**: The $< 5\%$ target remains elusive in purely MEMS-based systems during 60s blackouts due to unobservable yaw heading drift exceeding the geometric constraints offered by typical road segments when GNSS covariance expands.

---

## 2. GNSS+INS Fusion Update Rate & Wall-Clock Throughput

| Platform | Target Rate | Measured Latency (Mean) | Benchmark Status |
| :--- | :--- | :--- | :--- |
| **Mobile App** | 10.0 Hz | 1.581 ms | **PASS** |
| **Edge Engine** | ~200.0 Hz | 1.113 ms | **PASS** |
