
<img width="1093" height="594" alt="螢幕錄影 2026年八月06日 10時03分01秒" src="https://github.com/user-attachments/assets/59a6349d-5fe4-4d5d-a007-f7fb5eac061a" />




這份模擬程式是一個基於網頁（HTML5 Canvas）的**單軸 PID 控制與反應頻率實驗台**。以下數學式詳細梳理該模擬程式背後的物理模型、離散 PID 控制器架構以及數位控制的時間同步機制：

---

### 一、 物理系統與旋轉運動方程式

模擬中的旋轉機臂可視為一個受力矩作用的單軸旋轉系統。根據牛頓第二運動定律（旋轉版），其角加速度 $\alpha$ 與受到的總力矩 $\tau_{\text{total}}$ 關係為：

$$J \alpha = \tau_{\text{total}}$$

其中：
* $J$ 為**轉動慣量**（單位： $$\text{kg}\cdot\text{m}^2$$ ），程式中預設設為 $J = 0.025$。
* $\alpha = \frac{d\omega}{dt}$ 為角加速度（單位： $$\text{rad}/\text{s}^2$$ ）。
* $\omega$ 為當前的**角速度**（單位： $$\text{rad}/\text{s}$$ ）。
* 角度 $\theta$ 的變化率即為角速度：
  $$\frac{d\theta}{dt} = \omega$$

#### 總力矩 $\tau_{\text{total}}$ 的組成：
系統在每個瞬間受到的總力矩包含控制器輸出的主動控制力矩 $\tau_{\text{control}}$ 以及系統本身的微小自然阻尼（與角速度成正比）：

$$\tau_{\text{total}} = \tau_{\text{control}} - b \omega$$

* $b$ 為**自然阻尼係數**（程式中預設 `base_b = 0.005`）。

---

### 二、 離散 PID 控制器數學模型

在數位控制系統中（如模擬 RP2350 微控制器的執行緒），PID 計算是以固定的控制週期 $T_s = \frac{1}{f_{\text{control}}}$ 進行。

1. **誤差計算（Error）：**
   設目標角度為 $\theta_{\text{target}}$（由度數轉換為弳度），當前角度為 $\theta$，則離散時間點 $k$ 的誤差 $e[k]$ 為：
   $$e[k] = \theta_{\text{target}} - \theta[k]$$

2. **比例項（Proportional, $P$）：**
   $$P[k] = K_p \cdot e[k]$$
   * $K_p$ 相當於彈簧剛度。

3. **積分項（Integral, $I$）：**
   採用數值積分（矩形法），並具有防飽和機制（Anti-windup，限制在 $[-2.0, 2.0]$ 之間）：
   $$\text{Integral}[k] = \text{Integral}[k-1] + e[k] \cdot T_s$$
   $$\text{Integral}[k] = \min(\max(\text{Integral}[k], -2.0), 2.0)$$
   $$I[k] = K_i \cdot \text{Integral}[k]$$

4. **微分項（Derivative, $D$）：**
   採用標準離散後向差分（Backward Difference）：
   $$D[k] = K_d \cdot \frac{e[k] - e[k-1]}{T_s}$$
   * 此項扮演物理阻尼的角色[cite: 1]。

5. **控制器總輸出（力矩）：**
   $$\tau_{\text{control}}[k] = P[k] + I[k] + D[k]$$
   此控制力矩會維持定值，直到下一個控制週期到來。

---

### 三、 時間同步與多速率架構 (Multi-rate Architecture)

模擬程式的核心亮點在於分離了**「控制運算頻率 ($f_{\text{control}}$)」**與**「繪圖渲染更新率 ($60\,\text{fps}$ 畫布動畫)」**。其運作機制如下：

* 藉由時間累積器（`control_accumulator`）計算實際流逝的時間 $\Delta t$：
  $$\text{accumulator} \leftarrow \text{accumulator} + \Delta t$$
* 當累積時間大於或等於離散控制週期時（即 $\text{accumulator} \ge \frac{1}{f_{\text{control}}}$）：
  * 觸發一次上述的**離散 PID 核心運算**。
  * 累積器扣除一個週期： $$\text{accumulator} \leftarrow \text{accumulator} - \frac{1}{f_{\text{control}}}$$ 。
* 在每一個動畫畫格（Animation Frame）中，物理世界則以高頻率的 $\Delta t$ 進行歐拉積分更新（Euler Integration）：
  $$\omega^{(n+1)} = \omega^{(n)} + \left(\frac{\tau_{\text{control}} - b \omega^{(n)}}{J}\right) \Delta t$$
  $$\theta^{(n+1)} = \theta^{(n)} + \omega^{(n)} \Delta t$$

這樣的數學與程式架構，能真實呈現當**控制反應頻率（ $$f_{\text{control}}$$ ）**過低時，因控制更新延遲所造成的系統震盪與發散現象。

---

## 四、 震盪現象與二階系統理論的關聯

在調校過程中產生的**震盪現象**，可從標準二階系統的特徵方程式 $s^2 + 2\zeta\omega_n s + \omega_n^2 = 0$ 得到解釋：

1. **自然頻率 $\omega_n$**：
   $$\omega_n = \sqrt{K \cdot Kp_{in} \cdot Kp_{out}}$$
   * $\omega_n$ 決定了系統的反應速度與剛性。當增益開太大時 $\omega_n$ 增大，若阻尼不足會導致劇烈的高頻震盪。
2. **阻尼比 $\zeta$**：
   $$\zeta = \frac{b_{aero}}{2I\omega_n} + \frac{K \cdot Kp_{in}}{2\omega_n}$$
   * 阻尼比由「被動氣動阻尼項 ($\zeta_p$)」與「內環主動回授阻尼項 ($\zeta_a$)」共同組成。
   * 當 $\zeta < 1$（欠阻尼）時，系統會出現過衝與擺動震盪；若調降 $Kp$ 或增加內環微分 $Kd_{in}$ 強化主動阻尼，則能有效抑制震盪。
3. **實際震盪頻率（阻尼震盪頻率 $\omega_d$）與自然頻率的差異**：
   * 實際發生震盪時的頻率並**不完全等於**自然頻率 $\omega_n$，而是稱為**阻尼震盪頻率（Damped Frequency, $\omega_d$）**。兩者的數學關係為：
     $$\omega_d = \omega_n \sqrt{1 - \zeta^2}$$
   * **物理意義**：自然頻率 $\omega_n$ 是指系統在完全沒有阻尼（$\zeta = 0$）時的固有振動頻率。但在實際系統中，由於機構天生的阻尼與內環 PID 提供的主會回授阻尼存在，阻尼會將震盪拉慢、使週期變長，因此實際觀察到的震盪頻率 $\omega_d$ 會**略低於**自然頻率 $\omega_n$。只有在 $\zeta = 0$ 的極端理想情況下，兩者才會相等。

---

## 五、 數位控制時間同步機制與多速率架構詳解

模擬程式透過時間累積器（`control_accumulator`）成功將**控制運算頻率 ($f_{\text{control}}$)** 與 **60 fps 繪圖渲染頻率**進行了解離與同步，其運作細節、多速率架構與「細分積分」機制如下：

* **非同步與解耦設計**：在實際的嵌入式微控制器（如 RP2350）中，PID 控制迴圈是以固定迴圈週期精準執行的。然而，網頁端的 `requestAnimationFrame` 畫面渲染頻率受限於螢幕更新率（約 60 fps），其時間間隔可能隨瀏覽器效能波動。透過時間累積器，模擬能夠在獨立於繪畫幀率的時脈下執行數位控制。
* **時間累積器（`control_accumulator`）運作邏輯**：在每一個動畫畫格中，程式會先計算實際流逝的真實物理時間 $\Delta t$（程式中的 `elapsed`），並將其累加至累積器中（`control_accumulator += elapsed`）。
* **離散控制週期的觸發條件**：當累積時間大於或等於控制週期 $T_s = \frac{1}{f_{\text{control}}}$ 時，程式才會觸發一次雙環 PID 核心計算。若單一畫格間隔較長，`while` 迴圈會連續執行多次 PID 計算以補足時間差，確保控制頻率的數學正確性。
* **零階保持器（ZOH, Zero-Order Hold）特性的重現**：在兩次控制計算的間隔空檔中，內環 PID 算出的最後力矩（`current_control_torque`）會維持固定不變，這完全符合實際數位控制器透過輸出控制訊號時的物理行為。
* **細分積分（Sub-stepping / Subdivided Integration）機制**：
  * **定義與運作**：物理模擬的世界與運動學計算，會隨著每一個畫面更新畫格（Frame）所流逝的微小時間步長 $\Delta t$ 進行連續的數值計算（如歐拉積分）。
  * **與控制頻率的區分**：控制器的 PID 計算是依照設定的離散頻率（如 $50\,\text{Hz}$）間歇性執行的；而物理世界的動態（如轉動慣量、受力矩後的角加速度、角度與速度的變化）則是在每一個畫面渲染週期中，利用實際流逝的時間間隔 $\Delta t$ 進行連續更新。
  * **工程意義**：透過這種細分積分機制，網頁不僅能維持平順的 60 fps 動畫視覺，還能精準對應數學模型，真實模擬出當控制頻率過低或增益過高時，系統在連續物理時間軸上所產生的相位落後與發散震盪現象。
