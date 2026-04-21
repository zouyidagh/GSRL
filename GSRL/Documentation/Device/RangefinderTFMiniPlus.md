# [cite_start]北醒 Benewake [cite: 1]

# [cite_start]TFmini Plus 用户手册 [cite: 2]

### [cite_start]北醒(北京)光子科技有限公司 [cite: 3]

-----

# [cite_start]前言 [cite: 4]

[cite_start]尊敬的用户: [cite: 5]
[cite_start]您好,感谢您选择北醒的产品。为让产品的使用体验更好,我们特制定此用户手册,以帮助您更加便捷地使用产品。 [cite: 6]

[cite_start]本用户手册包括TFmini Plus 激光雷达测距模组的介绍、使用和维护等相关内容,涵盖常见情况下的使用说明及问题处理措施。请在使用前仔细阅读本手册,谨记注意事项,避免危险,并在使用过程中,严格遵守手册内所述步骤执行。 [cite: 7]

[cite_start]如果您在使用过程中遇到了无法解决的问题,欢迎您随时联系北醒工作人员协助解决。 [cite: 8]

[cite_start]**联系方式** [cite: 9]

  * [cite_start]官网地址: [www.benewake.com](https://www.benewake.com) [cite: 10]
  * [cite_start]联系电话: 400-880-9610 [cite: 11]
  * [cite_start]咨询技术问题,请联系: support@benewake.com [cite: 12]
  * [cite_start]咨询销售事宜或索取介绍资料,请联系: bw@benewake.com [cite: 13]

[cite_start]**公司总部地址** [cite: 14]

  * [cite_start]北醒(北京)光子科技有限公司 [cite: 15]
  * [cite_start]北京市海淀区上地街道海国嘉业科技园3层 [cite: 16]

[cite_start]**版权声明** [cite: 17]
[cite_start]本文档版权归©北醒公司所有,未经北醒公司的官方书面许可,请勿改变文档中的内容描述,以及对文档进行修改、删减或翻译。 [cite: 18]

[cite_start]**免责声明** [cite: 19]
[cite_start]我们的产品还在不断改进和更新,因此TFmini Plus 的规格参数可能会发生变化,请以官网上的最新版本为准。 [cite: 20]

[cite_start]**产品认证** [cite: 21]

  * [cite_start]IEC [cite: 22]
  * [cite_start]ROHS [cite: 23]
  * [cite_start]EN62471光生物安全认证 [cite: 24]
  * [cite_start]CE [cite: 25]

-----

# [cite_start]目录 [cite: 26]

| 章节 | 标题 | 页码 |
| :--- | :--- | :--- |
| **1** | **概览** | **1** |
| 1.1 | 产品参数 | 1 |
| **2** | **注意事项** | **1** |
| 2.1 | 关于文档 | 1 |
| 2.2 | 产品使用 | 1 |
| 2.3 | 产品失效情况 | 1 |
| **3** | **功能及关键参数** | **2** |
| 3.1 | 产品功能 | 2 |
| 3.2 | 测距原理 | 2 |
| 3.3 | 外观与结构 | 2 |
| **4** | **线序与数据通信协议** | **4** |
| 4.1 | 线序说明 | 4 |
| 4.2 | 串口数据通信 | 4 |
| 4.3 | 串口数据输出格式及编码 | 4 |
| 4.4 | 输出数据说明 | 5 |
| 4.5 | I2C 数据通信 | 5 |
| **5** | **快速测试步骤** | **7** |
| 5.1 | 产品测试所需工具 | 7 |
| 5.2 | 测试步骤 | 7 |
| **6** | **自定义参数配置说明** | **9** |
| 6.1 | 功能简介 | 9 |
| 6.2 | 配置指令通信约定 | 9 |
| 6.3 | 帧定义 | 9 |
| 6.4 | 一般参数配置及说明 | 9 |
| **7** | **远程升级** | **12** |
| **8** | **故障-原因和处理措施** | **13** |
| **9** | **常见问题及解答** | **14** |
| **附录** | **TF 系列上位机使用说明** | **15** |
[cite_start][cite: 27]

-----

# [cite_start]1 概览 [cite: 31, 32, 33]

[cite_start]TFmini Plus 是基于TFmini的升级项目,它是一款小型化,单点测距的产品,基于ToF(飞行时间)原理,配合独特的光学、电学、算法设计,主要实现稳定、精准、高灵敏度和高速的距离测量的功能。 [cite: 34]

## [cite_start]1.1 产品参数 [cite: 35]

[cite_start]**表1-1 TFmini Plus 产品参数列表** [cite: 36]

| 参数大类 | 参数名称 | 参数值 |
| :--- | :--- | :--- |
| **产品性能** | 测量范围 (90%反射率) | 0.1m\~12m |
| | 测量范围 (10%反射率) | 0.1m\~4m |
| | 准确度① | ±5cm (0.1m\~5m), ±1% (5m\~12m) |
| | 距离分辨率 | 1cm |
| | 帧率② | 100Hz |
| | 抗环境光能力 | 70Klux |
| | 防护等级 | IP65 |
| | 人眼安全 | 豁免级(EN62471) |
| **光学参数** | 中心波长 | 850nm |
| | 光源 | LED |
| | 视场角③ | 3.6° |
| **电学参数** | 供电电压 | DC 5V±0.5V |
| | 通信电平 | LVTTL (3.3V) |
| | 平均电流 | ≤110mA |
| | 功耗 | 550mW(低功耗模式:\<100mW) |
| | 峰值电流 | 140mA |
| **其他** | 尺寸 | 35mm\*18.5mm\*21mm(长\*宽\*高) |
| | 壳体材质 | ABS/PC |
| | 工作温度 | $-20^{\circ}C\sim60^{\circ}C$ |
| | 存储温度 | $-20^{\circ}C\sim75^{\circ}C$ |
| | 重量 | 12g±1g |
| | 线长 | 30cm |
[cite_start][cite: 37]

[cite_start]**提示:** [cite: 43]
[cite_start]① 准确度、距离分辨率、重复精度等均在室内25℃ 漫反射白板(90%反射率)条件下测得。 [cite: 45, 46]
[cite_start]② 帧数可调整,最高支持1000Hz。 [cite: 47]
[cite_start]③ 视场角为理论值,实际角度值存在一定偏差。 [cite: 48]

-----

# [cite_start]2 注意事项 [cite: 52, 54]

## [cite_start]2.1 关于文档 [cite: 55]

[cite_start]本说明书提供产品使用过程中必需的各项信息。 [cite: 56]
[cite_start]请在使用本产品前认真阅读本说明书,并确保您已完全理解说明书内容。 [cite: 57]

## [cite_start]2.2 产品使用 [cite: 58]

  * [cite_start]本产品只能由合格的专业人员维修,且只能使用原厂备件,以保证产品的性能和安全性。 [cite: 59]
  * [cite_start]产品本身无极性保护和过电压保护,请按说明书内容正确接线和供电。 [cite: 60]
  * [cite_start]产品的工作温度为 $-20^{\circ}C\sim60^{\circ}C$,请勿在此温度范围外使用,以免产生风险。 [cite: 61]
  * [cite_start]产品的存储温度为 $-20^{\circ}C\sim75^{\circ}C$,请勿在此温度范围外存储,以免产生风险。 [cite: 62]
  * [cite_start]请勿打开外壳进行本使用说明以外的装配或保养,以免影响产品防护性能,造成产品失效。 [cite: 63]

## [cite_start]2.3 产品失效情况 [cite: 64]

  * [cite_start]产品在探测高反射率物体,如镜面、光滑地砖、平静的牛奶液面时,会有失效的风险。 [cite: 65]
  * [cite_start]当产品与被测目标之间有透明物体,如玻璃、水时,会有失效的风险。 [cite: 66]
  * [cite_start]当产品发射接收窗口被污物覆盖时,会有失效的风险,请保持窗口干净。发射接收窗口为红透亚克力材质,请勿让产品接触酒精,会导致产品损坏。 [cite: 67, 68]
  * [cite_start]本产品线缆较细,请在使用时不要用力拉拽线缆,会导致产品损坏。 [cite: 69]

-----

# [cite_start]3 功能及关键参数 [cite: 75, 76]

## [cite_start]3.1 产品功能 [cite: 77]

[cite_start]TFmini Plus 是基于TFmini 的升级项目,它是一款小型化,单点测距的产品,基于ToF(飞行时间)原理,配合独特的光学、电学、算法设计,主要实现稳定、精准、高灵敏度和高速的距离测量的功能。产品本身除了具有TFmini的低成本、小体积、测距远等特点外,还增加了IP65等级防护,测距精度更高,对于室外强光、不同温度、不同反射率等不同环境下适应性更强,更低功耗,探测频率也更加灵活。产品同时兼容UART和12C通信接口,可通过指令进行切换。 [cite: 78]

## [cite_start]3.2 测距原理 [cite: 79]

[cite_start]TFmini Plus 基于ToF (Time of Flight)即飞行时间原理。具体为产品周期性的向外发出近红外光调制波,调制波遇物体后反射。产品通过测量调制波往返相位差,得到飞行时间,再计算出产品与被测目标之间的相对距离,如图1所示。 [cite: 80]
[cite_start]*公式图示参考: $D=\frac{c}{2} \cdot \frac{1}{2\pi f} \cdot \Delta\varphi$ (c为光速)* [cite: 85, 90]
[cite_start]**图3-1 飞行时间原理示意图** [cite: 86]

## [cite_start]3.3 外观与结构 [cite: 84]

[cite_start]**图3-2 TFmini Plus 尺寸图** [cite: 93]
[cite_start]*(尺寸参数: 35mm 长, 18.5mm 宽, 21mm 高, 安装孔距 28mm)* [cite: 87, 88, 91, 92]

## [cite_start]3.4 测距特性 [cite: 99]

[cite_start]**图3-3 产品测距范围及有效性示意图** [cite: 116]

[cite_start]TFmini Plus 产品经过光路与算法优化,已最大程度减小外界环境对测距性能的影响。但限于工作原理,测距范围仍会受到环境光照强度和被测目标反射率不同程度的影响。如图2所示: [cite: 117]

  * [cite_start]序号①: 代表TFmini Plus 的测距盲区,为0-10cm,该范围内的数据不可信。 [cite: 117]
  * [cite_start]序号②: 代表TFmini Plus 对黑色(10%反射率)目标的探测能力,测量范围为0.1-4m。 [cite: 118]
  * [cite_start]序号③: 代表TFmini Plus 对白色(90%反射率)目标的探测能力,测量范围为0.1-12m。 [cite: 118]

[cite_start]纵坐标: 代表不同距离下TFmini Plus 的有效测距边长,只有当『被测目标边长』大于等于“有效测距边长”时,数据才稳定可靠。该『有效测距边长』由TFmini 的视场角决定(视场角一般是指接收角和发射角中的较小者),计算公式为: [cite: 118]

[cite_start]$d=2*D\cdot tan\beta$ [cite: 119]

[cite_start]其中,d表示有效测距边长,D表示探测距离,$\beta$为TFmini Plus 的接收半角1.8°,一般的有效测距边长与探测距离的对应关系,见表3-1 [cite: 120]

[cite_start]**表3-1 测距距离对应的被测目标有效边长** [cite: 121]

| 探测距离(m) | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **有效边长(cm)** | 6 | 12 | 19 | 25 | 31 | 38 | 44 | 50 | 57 | 62 | 69 | 76 |
[cite_start][cite: 122]

[cite_start]当被测物体边长不满足有效测距边长时,如图3-4所示,TFmini Plus 输出测量值(Dist)会出现异常。使用过程中如果要求精度较高,应尽量避免此类情况,减小测量误差。 [cite: 123]

[cite_start]**图3-4 探测两个距离不一的物体 (Dist2 \> Dist \> Dist1)** [cite: 125, 126]

-----

# [cite_start]4 线序与数据通信协议 [cite: 135]

## [cite_start]4.1 线序说明 [cite: 136]

[cite_start]**图4-1 TFmini Plus 外部接线端子及线序** [cite: 140]

[cite_start]**表4-1 引脚功能及连接说明** [cite: 141]

| 编号 | 颜色 | 对应 PIN 脚 | 功能 | 说明 |
| :--- | :--- | :--- | :--- | :--- |
| ① | 红 | PIN-1 | +5V | 电源正极 |
| ② | 白 | PIN-2 | RXD/SDA | 接收/数据 |
| ③ | 绿 | PIN-3 | TXD/SCL | 发送/时钟 |
| ④ | 黑 | PIN-4 | GND | 电源地 |
[cite_start][cite: 142]

[cite_start]产品连接线长30cm,连接器为普通 GH1.25-4p (Molex51021-0400)。客户可自行延长连接线,为保证数据的有效传输,建议自行焊接的连接线长度不大于1m。线缆功用以上图中颜色信息为准。 [cite: 143]

## [cite_start]4.2 串口数据通信 [cite: 144]

[cite_start]TFmini Plus 串口数据通信,详见表4-2 [cite: 145]

[cite_start]**表4-2 TFmini Plus 数据通信协议——UART** [cite: 145]

| 接口参数 | 默认值 |
| :--- | :--- |
| 默认波特率 | 115200 |
| 数据位 | 8 |
| 停止位 | 1 |
| 奇偶校验 | None |
[cite_start][cite: 146]

## [cite_start]4.3 串口数据输出格式及编码 [cite: 147]

[cite_start]TFmini Plus 有两种数据输出格式,标准数据输出格式和字符串数据格式,两种格式可通过指令代码相互切换。 [cite: 148, 149]

[cite_start]**标准数据输出格式(默认):** [cite: 150]
[cite_start]数据结构:数据帧长度为9字节。包含距离信息(Distance)、信号强度信息(Strength)、温度(Temp)、数据校验字节(Checksum)等。数据格式为16进制(HEX)。具体数据编码详见表4-3。 [cite: 155]

[cite_start]**表4-3 数据格式及编码解释** [cite: 156]

| Byte0-1 | Byte2 | Byte3 | Byte4 | Byte5 | Byte6 | Byte7 | Byte8 |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| 0x59 59 | Dist\_L | Dist\_H | Strength\_L | Strength\_H | Temp\_L | Temp\_H | Checksum |

**数据编码解释**

  * **Byte0**: 0x59,帧头,每一帧都相同
  * **Byte1**: 0x59,帧头,每一帧都相同
  * **Byte2**: Dist L距离值低八位
  * **Byte3**: Dist H距离值高八位
  * **Byte4**: Strength\_L低八位
  * **Byte5**: Strength\_H 高八位
  * **Byte6**: Temp\_L低八位
  * **Byte7**: Temp\_H 高八位
  * **Byte8**: Checksum 为前8字节数据的累加和,取累加和的低8位
    [cite_start][cite: 157]

[cite_start]**字符串数据格式** [cite: 158]
[cite_start]以字符串形式输出,单位为m,比如测距为1.21m,则输出字符串1.21,后跟转义字符\\r\\n。 [cite: 159]

## [cite_start]4.4 输出数据说明 [cite: 160]

  * [cite_start]**Dist(Distance)**: 代表 TFmini Plus 测量输出的距离值,默认单位为cm,解析为十进制的值范围为0-1200。实际使用过程中,当信号强度值 Strength\<100或等于65535(信号过曝)时,Dist 的测量值被认为不可信,默认输出0。 [cite: 161]
  * [cite_start]**Strength**: 指信号强度,默认输出值会在0-65535之间。当测距档位一定时,测距越远,信号强度越低;目标物反射率越低,信号强度越低。当Strength 大于100且不等于65535时,认为 Dist 的测量值可信,客户可以根据使用场景自行调整。 [cite: 162]
  * [cite_start]**Temp(Temperature)**: 表征芯片内部温度值。摄氏度=Temp/8-256 [cite: 163]

## [cite_start]4.5 12C 数据通信 [cite: 164]

[cite_start]TFmini Plus 同时支持12C数据通信接口,见下表: [cite: 165]

[cite_start]**表4-4 TFmini Plus 数据通信协议——I²C** [cite: 166]

| 通信接口 | I2C |
| :--- | :--- |
| 最大传输速率 | 400kbps |
| 主从模式 | 从机模式 |
| 默认地址 | 0x10 |
| 地址范围 | 0x01\~0x7F |
[cite_start][cite: 167]

[cite_start]**I2C模式数据时序说明** [cite: 172]
[cite_start]与串口通信不同,I2C通信由主机发起,雷达作为从机只能被动收发数据。主机向雷达写入配置指令帧后,需要等待足够长的时间,等待处理完该指令后,再进行读取反馈操作,建议等待时间为100ms。为保证测距状态下的实时性,“获取测距结果”数据帧无需设置等待时间,主机发送下行帧后,可以立即读取上行帧。详见下表: [cite: 173]

[cite_start]**表4-5 TFmini Plus 12C模式通信时序** [cite: 174]
[cite_start]*写入时序:* Start -\> Addr -\> WA -\> Byte0 -\> A -\> ... -\> ByteN -\> A -\> Stop [Wait 100ms] [cite: 175]
[cite_start]*读取时序:* Start -\> Addr -\> RA -\> Byte0 -\> A -\> ... -\> ByteN -\> A -\> Stop [cite: 175]

[cite_start]**I/O 模式说明** [cite: 176]
[cite_start]本产品增加I/O输出模式支持,可通过相关指令使能该模式。详见7.4。指令开放模式(Mode),临界距离值(Dist)及滞回区间(Zone)三个可配置参数: [cite: 177]

  * [cite_start]**Mode**: 0(数据输出模式),1(开关量模式,近高远低),2(开关量模式,近低远高);默认值为0 [cite: 178]
  * [cite_start]**Dist**: 临界值,滞回区间的近端点值,单位cm;默认值为0 [cite: 179]
  * [cite_start]**Zone**: 滞回区间大小,单位cm;默认值为0(无滞回区间) [cite: 180]

[cite_start]通过该指令设置开关临界区的滞回区间,当输出为近区电平时,测量值大于滞回区间的远端点后,输出切换为远区电平;当输出为远区电平时,测量值小于滞回区间的近端点时,输出切换为近区电平。(高电平: 3.3V,低电平:0V) [cite: 181]

[cite_start]**注:** 使用I/O模式前,请将低阈值默认输出值修改为1200(发送指令: `5A 07 22 0A B0 04 00 5A 04 11 6F`),可以避免远距离误报的问题。 [cite: 182]

-----

# [cite_start]5 快速测试步骤 [cite: 187]

## [cite_start]5.1 产品测试所需工具 [cite: 188, 189]

  * [cite_start]TFmini Plus [cite: 191]
  * [cite_start]TTL - USB 板 [cite: 191]
  * [cite_start]USB 线 [cite: 192]
  * [cite_start]电脑 [cite: 193]
  * [cite_start]上位机软件 [cite: 194]

## [cite_start]5.2 测试步骤 [cite: 195]

[cite_start]**(1) 上位机测试软件下载** [cite: 196]
([http://www.benewake.com/download](http://www.benewake.com/download))下载 TFmini Plus 上位机软件。
[cite_start]注意:解压上位机软件前请关闭杀毒软件,避免上位机软件中的文件被当成病毒删除,上位机目前仅支持在 Windows 系统上运行。详见附录一:《TF上位机使用说明》。 [cite: 197]

[cite_start]**(2) 设备连接** [cite: 197]
[cite_start]**图5-1 正确连接示意图** [cite: 198]
[cite_start]如上图所示,连接『TFmini Plus』、『TTL-USB转接板』和『USB线』,确保无松动,再将『USB线』与『电脑』连接。 [cite: 199]

[cite_start]**(3) 上位机连接与读数** [cite: 200]
[cite_start]如图,打开TF上位机,选择『TFmini Plus』,并选择自动识别的占用串口(这里是『② COM57』)。 [cite: 201]
[cite_start]然后,点击『CONNECT』进行上位机连接。连接成功后,右侧『④ TIME LINE CHART』区域会出现连续输出的数据图像,下方『⑥ REAL TIME DATA』区实时显示当前测试距离(Dist)、每秒有效数据量(Effective Points)和信号强度(Strength)。 [cite: 206]

[cite_start]**图5-2 上位机界面及显示** [cite: 250]

[cite_start]**说明:** [cite: 251]

  * [cite_start]如果『④ TIME LINE CHART』区没有数据,请检查连接和线序,TFmini Plus上电成功,正面看发射透镜内会有微弱的红光。 [cite: 252]
  * [cite_start]如果 TFmini Plus 是 Pixhawk格式输出,需先勾选『③ Pix Mode』,『TIME LINE CHART』区才会正常输出数据图像。勾选Pix Mode后,距离单位变为m。 [cite: 253]
  * [cite_start]距离输出 Dist值,跟据输出单位不同会有所区别,默认单位为cm。如果通过指令修改 TFmini Plus 的距离单位为mm,上位机并不能区分,『TIME LINE CHART』单位仍为cm。例如,TFmini Plus 实际测量距离为1m,以mm为单位则输出1000,通过该上位机读取的数值为1000,但上位机上的单位不会变化,仍显示cm。 [cite: 254]

-----

# [cite_start]6 自定义参数配置说明 [cite: 259]

## [cite_start]6.1 功能简介 [cite: 260]

[cite_start]为了让 TFmini Plus可以更灵活的解决您的问题,特开放用户自定义配置产品参数的功能。用户可通过发送相关指令来修改产品的原有参数,如输出数据格式、输出帧率等。 [cite: 261, 262]
[cite_start]请根据需求修改产品配置,切勿频繁尝试不相关指令,以免指令发送错误造成不必要的损失;请务必按照本说明书所列指令进行产品配置,切勿发送未声明的指令。 [cite: 263]

## [cite_start]6.2 配置指令通信约定 [cite: 264]

[cite_start]多字节数据采用小端模式传输,即数据的低字节保存在数据帧的低地址中。 [cite: 265]
[cite_start]如,十进制数1000对应十六进制为0x03E8,则在数据帧保存为 `0x5A 0x06 0x03 0xE8 0x03 0x4E` [cite: 266, 267]

## [cite_start]6.3 帧定义 [cite: 268]

[cite_start]注意:所有配置指令均为16进制数(HEX)发送。 [cite: 269]

[cite_start]**表6-1 指令编码格式及详细描述** [cite: 270]

| Byte0 | Byte1 | Byte2 | Byte3 \~ ByteN-2 | ByteN-1 |
| :--- | :--- | :--- | :--- | :--- |
| Head | Len | ID | Payload | Checksum |

**指令编码解释**

  * **Byte0**: Head: 指令帧的帧头(固定值,0x5A)
  * **Byte1**: Len: 指令帧总长度(包含Head 和 Checksum,单位为字节)
  * **Byte2**: ID: 代表不同功能指令的解析方式
  * **Byte3-N-2**: Data: 数据段,根据ID进行解析,数据为小端格式
  * **ByteN-1**: Checksum: 对从 Head 到 Payload 的所有字节进行求和计算,取低8位
    [cite_start][cite: 271]

## [cite_start]6.4 一般参数配置及说明 [cite: 272]

[cite_start]设置 TFmini Plus的相关参数,请先将TFmini Plus 与PC建立连接,连接方式参考6.2,通过TF 上位机或者其他串口调试软件,给产品发送相关配置指令;客户也可以通过自己的串口工具发送相关指令。所有指令在UART及IIC模式下通用。 [cite: 273]

[cite_start]**重要:** 在发送完参数配置指令后,请务必发送"保存配置”指令,否则再次连接产品时,参数将重置。 [cite: 274]

[cite_start]**表6-2 一般参数配置指令列表** [cite: 278]

| 可配置项 | 下行指令 | 上行指令 | 说明 | 出厂配置 |
| :--- | :--- | :--- | :--- | :--- |
| 获取固件版本 | `5A 04 01 5F` | `5A 07 01 V1 V2 V3 SU` | 版本 V3.V2.V1 | / |
| 系统复位 | `5A 04 02 60` | `5A 05 02 00 60`<br>`5A 05 02 01 61` | 配置成功<br>配置失败 | / |
| 输出帧率 | `5A 06 03 LL HH SU` | `5A 06 03 LL HH SU` | 1-1000Hz 设置① | 100Hz |
| 单次触发指令 | `5A 04 04 62` | 数据帧<br>`5A 05 05 01 65` | 将输出帧率设置为0后,可通过本指令出发测试 | √ |
| 输出模式 | `5A 05 05 01 65` | `5A 05 05 01 65` | 标准9字节(cm) | √ |
| | `5A 05 05 02 66` | `5A 05 05 02 66` | 字符串格式(m) | / |
| | `5A 05 05 06 6A` | `5A 05 05 06 6A` | 标准9字节(mm) | / |
| 波特率 | `5A 08 06 H1 H2 H3 H4 SU` | `5A 08 06 H1 H2 H3 H4 SU` | 设置波特率②<br>例: 256000(DEC)=3E800(HEX)<br>H1=00,H2=E8,H3=03,H4=00 | 115200 |
| 输出开关 | `5A 05 07 00 66`<br>`5A 05 07 01 67` | `5A 05 07 00 66`<br>`5A 05 07 01 67` | 关闭数据输出<br>使能数据输出 | /<br>√ |
| 通信接口设置 | `5A 05 0A MODE SU` | / | 0 (UART)<br>1 (I2C) | UART |
| 修改 I2C 从机地址 | `5A 05 0B ADDR SU` | 原指令 | 修改 I2C\_slave\_addr | 0x10 |
| 获取测距结果 | `5A 05 00 01 60`<br>`5A 05 00 06 65` | 数据帧(标准9字节(cm))<br>数据帧(标准9字节(mm)) | 仅IIC模式下可用 | / |
| I/O(开关量)模式使能 | `5A 09 3B MODE DL DH ZoneL ZoneH SU` | / | 开启/关闭I/O(开关量)输出模式<br>MODE:<br>0 - 标准数据模式<br>1 - I/O,近高远低<br>2 - I/O,近低远高<br>Zone:滞回区间 | 0(标准数据模式) |
| 信号强度低阈值和低阈值输出值③ | `5A 07 22 XX LL HH 00` | `5A 07 22 XX LL HH SU` | 修改示例:Strength\<100后,Dist 输出值修改为1200。<br>XX=100/10=10(DEC)=0A(HEX)<br>1200(DEC)=4B0(HEX)<br>LL=B0, HH=04 | Strength\<100后,Dist 输出值为0 |
| 低功耗模式使能 | `5A 06 35 0X 00 SU` | `5A 06 35 0X 00 SU` | X(HEX)取值范围0\~A,低功耗模式下输出频率不支持超过10Hz;<br>X\>0时,低功耗模式使能;<br>X=0时,低功耗模式关闭④ | / |
| 恢复出厂设置 | `5A 04 10 6E` | `5A 05 10 00 6F`<br>`5A 05 10 01 70` | 配置成功<br>配置失败 | / |
| 保存设置 | `5A 04 11 6F` | `5A 05 11 00 70`<br>`5A 05 11 01 71` | 配置成功<br>配置失败 | / |
[cite_start][cite: 280, 283]

[cite_start]**解释说明:** [cite: 287]
黄色背景色'SU'代表校验和。
[cite_start]① 该配置项主要用于调整产品的输出频率。输出频率默认值为100Hz,支持自定义配置,可配置值满足1000/n(n为正整数);随着频率提高,数据输出稳定性会降低。 [cite: 288]
[cite_start]② 须使用常用波特率(9600/14400/19200/56000/115200/460800/921600)。当输出帧率较高时,建议使用高波特率以确保数据传输稳定。发送修改波特率指令后,需要保持通电,切换为目标波特率下,发送保存设置指令才能是更改生效。 [cite: 289]
[cite_start]③ 信号强度低阈值设置为小于100的数值后,当信号强度低于100时,测距值的波动性会变大。 [cite: 290]
[cite_start]在发送完相关参数配置指令后,请务必发生'保存设置'指令,否则重新上电后设置将无法生效。系统复位指令发送后,请保持通电并等待1s,否则有可能导致无法复位。 [cite: 290]
[cite_start]④ 从低功耗模式切换为正常功耗模式后,输出频率将于低功耗模式下一致,若仍需要100Hz 输出,需要在关闭低功耗模式后,手动设置输出频率为100Hz。 [cite: 291]

-----

# [cite_start]7 远程升级 [cite: 296]

[cite_start]TFmini Plus 支持远程升级,当用户产品不能满足当前的使用需求,且北醒官方有相应的固件更新后,用户可通过“TFmini Plus 远程升级上位机”更新产品固件。请联系技术支持人员获取远程升级上位机。 [cite: 298]

[cite_start]**图7-1 TFmini Plus 固件升级上位机** [cite: 307]

[cite_start]TFmini Plus 固件升级所需要的工具与快速测试步骤中描述的基本一致,同样需要TTL-USB 板建立 TFmini Plus与电脑的连接。 [cite: 308, 310]
[cite_start]连接好后,打开TFmini Plus 远程升级上位机,选择合适的端口,此处为『①COM8』。在『② 115200』处输入正确的波特率,点击『③CONNECT』,建立TFmini Plus 与上位机通信;点击『④ Open Bin』选择需要更新的固件文件,上方文本框中会显示该固件文件地址。然后点击『⑤ Download Bin』即可完成更新。『⑥』会显示固件更新信息。 [cite: 310]

[cite_start]**注意:** 上位机和固件需要放在纯英文路径的文件夹下,否则升级会失败。 [cite: 310]
[cite_start]**注:** 使用I/O模式前,请将低阈值默认输出值修改为1200(发送指令: `5A 07 22 0A B0 04 00 5A 04 11 6F`),可以避免远距离误报的问题。 [cite: 309]

-----

# [cite_start]8 故障-原因和处理措施 [cite: 314, 315]

[cite_start]**(1) 正常使用TFmini Plus 情况下,有时距离值会跳变为0。** [cite: 317]

  * [cite_start]**原因:** 由于测试环境不同(被测目标的反射率和环境光干扰等),TFmini Plus 探测的信号强度会受到不同程度的影响。为保证测量数据的可靠性和稳定性,TFmini Plus 内部做了算法剔除,当信号强度不足或过曝时,默认状态下TFmini Plus 的距离值会反馈为0,仅用于提示用户该数据不可信。 [cite: 318]
  * [cite_start]**处理措施:** 请您将此类数值当作触发信号,以保证在TFmini Plus 输出不可信数据时,您的系统可采用其他可信数据做下一步判断决策。 [cite: 319]

[cite_start]**(2) 雷达输出距离值与实际距离误差较大。** [cite: 320]

  * [cite_start]**原因①:** TFmini Plus 数据通信协议解析错误。 [cite: 321]
      * [cite_start]**处理措施:** 检查数据通信解析方式,如错误,请查看数据格式,调整解析方式。 [cite: 322]
  * [cite_start]**原因②:** 限于TFmini Plus 的物理原理,被测目标为高反射率(镜面、光滑瓷砖等)或透明(玻璃、水等)物质时,可能出现所述现象。 [cite: 323, 324]
      * [cite_start]**处理措施:** 请尽量避免在此种情况下使用。 [cite: 325]
  * [cite_start]**原因③:** 产品透镜处有杂物遮盖。 [cite: 326]
      * [cite_start]**处理措施:** 请用干燥的无尘布轻轻将杂物擦除。 [cite: 327]

[cite_start]**(3) TFmini Plus 没有数据输出。** [cite: 328]

  * [cite_start]**原因:** 产品出厂前会经过严格的审检,以保证出厂的产品都可正常使用。因此可能是运输或者使用过程中的意外情况导致工作异常。 [cite: 329]
  * **处理措施:**
      * [cite_start]检查供电是否正常,电压是否在额定电压范围内。如供电正常,TFmini Plus 发射镜头内会有微弱红光。 [cite: 330]
      * [cite_start]检查 TFmini Plus 接线顺序是否正确,连接是否可靠。 [cite: 331]
      * [cite_start]检查数据解析是否正确,请按照说明书说明的数据格式进行解析。 [cite: 332]
      * [cite_start]如仍未解决问题,请联系技术支持。 [cite: 333]

[cite_start]**(4) 雷达连接上位机后,无数据输出。** [cite: 334]

  * [cite_start]**原因①:** 目前上位机仅支持 Windows操作系统,其他系统暂不支持。 [cite: 335]
      * [cite_start]**处理措施:** 更换为Windows 操作系统的PC。 [cite: 336]
  * [cite_start]**原因②:** TTL-USB板连接不良。 [cite: 337]
      * [cite_start]**处理措施:** 检查TTL-USB板与 TFmini Plus 和PC的连接是否正确可靠。 [cite: 338]
  * [cite_start]**原因③:** 串口驱动未正确安装。 [cite: 339]
      * [cite_start]**处理措施:** 重新插拔 USB连接线,尝试重新安装驱动,或去网上直接搜索驱动程序下载安装。如果仍不能正常使用上位机,请联系我司技术支持。 [cite: 340, 341]

-----

# [cite_start]9 常见问题及解答 [cite: 344, 346, 347]

  * [cite_start]**Q1:请问TFmini Plus 是否支持3.3V或其他电压供电?** [cite: 348]
      * [cite_start]**A1:** 您好,目前不支持。TFmini Plus 标准供电5V±0.5V。如您有其他需求,可联系销售人员咨询定制事宜。 [cite: 349]
  * [cite_start]**Q2:请问TFmini Plus 工作一段时间后会发热,是坏了吗?** [cite: 350]
      * [cite_start]**A2:** 您好,这是产品正常工作状态。芯片与电路板持续工作后,轻微发热属于正常现象。 [cite: 351]
  * [cite_start]**Q3:请问 TFmini Plus 可以与 Arduino 或树莓派连接使用吗?** [cite: 352]
      * [cite_start]**A3:** 您好,可以。TFmini Plus 使用串口通信协议,只要是支持串口通信的控制板即可通信使用。 [cite: 354]
  * [cite_start]**Q4:请问2台TFmini Plus 同时工作会相互干扰吗?** [cite: 355]
      * [cite_start]**A4:** 您好,当2台TFmini Plus 同向摆放、光斑打在同一被测物上且重合的时候,不会互相干扰;当2台以上的TFmini Plus 同向摆放且光斑重合的时候,相互之间会有干扰;当2台TFmini Plus 面对面工作的时候,会产生严重的干扰。 [cite: 356]

-----

# [cite_start]附录- TF 系列上位机使用说明 [cite: 360]

## [cite_start]附录一 [cite: 361]

[cite_start]该上位机目前仅支持在 windows系统下使用,适用于北醒光子科技有限公司的TF系列产品,但仅限于按照串口通信协议输出的产品,TFmini Plus 具体操作细节见下列说明。 [cite: 362]

[cite_start]**图1 TF系列上位机界面** [cite: 419]

[cite_start]**1 产品型号/串口控制区【SETTINGS】** [cite: 420]

  * [cite_start]**Product Type 产品型号选择:** 在电脑端通过TTL-USB 转接板连接相应的雷达型号,如图使用的是本公司产品 TFmini Plus,选择TFmini Plus 即可。 [cite: 421]
  * [cite_start]**COM 串口通信的端口:** 选择电脑端识别雷达相应的端口号。TF系列产品默认波特率为115200,上位机中默认使用该波特率进行连接。 [cite: 422]
  * [cite_start]**CONNECT/DISCONNECT:** 点击【CONNECT】按钮,建立与雷达的连接;当点击【DISCONNECT】按钮,取消连接。 [cite: 423, 424]

[cite_start]**2 功能区【FUNCTION】** [cite: 425]

  * [cite_start]**Pix Mode 模式选择:** 如果是 Pixhawk 版本,勾选之后开启 PIX模式;取消勾选,恢复默认输出格式。请注意,因Pix模式输出格式特殊,此时上位机统计的实时帧率不可信。 [cite: 426]
  * [cite_start]**Frame Rate 更改帧率:** 点击下拉框,选择所需帧率,即时生效;可在【5】中有效点(Effective Points)处查看帧率变化。需注意,因数据传输问题,实际帧率会与理论帧率存在一定差别。 [cite: 427, 428]
  * [cite_start]**FREEZE/CLEAR 暂停/取消按钮:** 点击【FREEZE】之后,可以使上位机暂停,便于分析【4】中的图像;点击【CLEAR】之后,会清除【4】内的绘图曲线,重新开始绘图。 [cite: 433]
  * [cite_start]**Drawing/Pt 数据总计平均:** 默认是10,即上位机每接收10个数据,把10个点的数值取平均后在【4】内绘制一个点。可按需修改(为防止上位机卡顿,数值最好≥10),输入数值后,通过键盘回车键使能。 [cite: 434]
  * [cite_start]**Device Command 串口指令发送区:** 可通过此窗口对TFmini Plus 进行16进制串口指令的发送,需要注意的是输入指令完成后点击回车键,然后再点击上方的【SEND】按钮。 [cite: 435]

[cite_start]**3 数据录制区【DATA RECORDING】** [cite: 436]

  * [cite_start]**Record 数据录制栏:** 在文本窗口给要保存的数据命名,输入完毕后敲下回车键,通过【RECORD】按钮录取 TF 数据,数据会保存在命名的文本文件中,再次点击该按钮【FINISHED】,数据录制结束。 [cite: 437]
  * [cite_start]**FOLDER 打开文件夹:** 通过【FOLDER】打开数据保存的文件夹。 [cite: 438]
  * [cite_start]**注:** 当雷达输出帧率较高时,如1000Hz,因数据量较大上位机添加的时间戳存在不均匀现象。 [cite: 439]

[cite_start]**4 数据图像显示区【TIME LINE CHART】** [cite: 440]
[cite_start]上位机根据接收到的数据绘制连续的测距图像,纵坐标表示当前测距,横坐标表示有效点计数。 [cite: 440]

[cite_start]**5 实时数据显示区【REAL-TIME DATA】** [cite: 441]

  * [cite_start]**Dist 测距值:** 默认单位cm。 [cite: 442]
  * [cite_start]**Dist(Echo):** 此项为TF03产品参数,TFmini Plus 默认为0。 [cite: 443]
  * [cite_start]**Effective Points (per sec):** 表示TF每秒刷新的有效数据。 [cite: 444]
  * [cite_start]**Strength 信号强度:** 在pix模式下,由于没有强度输入 Strength 默认为0。 [cite: 445]

[cite_start]**6 使用环境及注意事项** [cite: 446]

  * [cite_start]**使用环境:** 本上位机需求 Windows 操作系统 Win7以上版本,同时PC中须安装 .Net Framework 4.5.2。 [cite: 447]
  * [cite_start]**注意事项:** 请勿将输出帧率大于500Hz的产品直接与上位机连接,会导致上位机界面卡死。 [cite: 448]
