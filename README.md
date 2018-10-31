# nRF52832-Gateway

## 一、功能概述
	BLE_Gateway模块(以下简称BLE_Gateway)扫描周围的BLE设备广播，获取BLE设备广播数据中的有效信息，通过串口发送到主MCU，
	另外可通过串口接收的AT指令查询和配置BLE_Gateway。
	串口波特率115200bps，8位数据位，1位停止位，无校验位，无硬件流控。
## 二、BLE广播有效信息分析
	BLE_Gateway获取广播中的设备名称、RSSI和MAC地址，广播中必须包含设备名称才会将解析出来的有效信息通过串口发送给主MCU。


## 三、用户自定义设备名称过滤规则
	BLE_Gateway也支持自定义设备名称过滤规则，最多支持20组，每组过滤规则有3个字符，详细介绍见串口AT指令章节。

## 四、串口AT指令
	BLE_Gateway支持的AT指令如下表所示。

|命令|说明|
|---|---|
|AT|测试串口通信
|AT+RESET|复位模块
|AT+MAC|查询模块MAC地址
|AT+VER|查询模块版本信息
|AT+FILTER|配置设备名称过滤规则使能
|AT+USER|配置具体的设备名称过滤规则

	所有AT指令(包括命令和响应)必须"AT"作为开头，以回车新行(<CR><LF>)结尾，为描述方便，文档中<CR><LF>被有意忽略了。


### 4.1 AT测试串口通信

|查询命令|响应|参数说明|
|---|---|---|
|AT?|AT:OK|无参数

### 4.2 AT+RESET复位模块

|设置命令|响应|参数说明|
|---|---|---|
|AT+RESET|AT+RESET:OK|无参数

### 4.3 AT+MAC查询模块MAC地址

|查询命令|响应|参数说明|
|---|---|---|
|AT+MAC?|AT+MAC:\<MAC address>|模块MAC地址(16进制字符串，12 Bytes)
	
### 4.4 AT+VER查询模块版本信息

|查询命令|响应|参数说明|
|---|---|---|
|AT+VER?|AT+VER:\<HW version>,\<FW version>,\<SW version>|\<HW version> 硬件版本<br>\<FW version> 协议栈固件版本<br>\<SW version> 应用软件版本

### 4.5 AT+FILTER 配置设备名称过滤规则使能

|设置命令|响应|参数说明|应用示例|
|---|---|---|---|
|AT+FILTER=\<enable>|发送成功 AT+FILTER:OK<br>参数错误 AT+FILTER:ERP|\<enable> 过滤规则使能,<br>1：使能，0：禁用|AT+FILTER=1<br>表示使能用户自定义过滤规则

### 4.6 AT+USER配置用户自定义广播过滤规则

|设置命令|响应|参数说明|应用示例|
|---|---|---|---|
|AT+USER=\<index>,\<en>,\<filter words>|发送成功 AT+USER:OK<br>参数错误 AT+USER:ERP|\<index> 自定义规则序号，最多20条，数值0~19有效<br>\<en> 自定义规则使能，1：使能，0：禁用<br>\<filter words> 设备名称过滤关键词，长度3字节|AT+USER=0,1,abc<br>表示自定义规则序号为0，<br>使能该自定义规则，<br>过滤关键字为"abc"
