#!/usr/bin/env python3
"""
STM32F407 + DAC8568 + ADS1256 上位机测试脚本
协议版本: 36B接收帧 / 66/82/86B发送帧
"""

import serial
import serial.tools.list_ports
import struct
import time
import threading
import sys

# ── 协议常量 ──
FRAME_HEADER = 0x68
FRAME_TAIL   = 0x16
RX_FRAME_LEN = 36    # PC→MCU 发送帧长
TX_STD_LEN   = 66    # 标准回复
TX_COL_LEN   = 82    # 采集回复
TX_SET_LEN   = 86    # 设置回复

# 命令码
CMD_DEVICE_CHECK  = 0x01  # 查询设备参数
CMD_LD_ENABLE     = 0x02  # 使能LD
CMD_LD_DISABLE    = 0x03  # 禁用LD
CMD_BIAS_V1_ENABLE= 0x06  # 使能偏压
CMD_CURRENT_SET   = 0x11  # 设置电流
CMD_BIAS_V1_SET   = 0x13  # 设置目标电压(mV)
CMD_LOOP_CHECK    = 0x18  # 查询实时数据
CMD_PID_SET       = 0x1C  # 设置PID参数
CMD_FF_SET        = 0x1D  # 前馈偏置(mV, x1000)
CMD_DAC_DIRECT    = 0x20  # 开环直接设DAC电压(mV), 0=恢复PID
CMD_WAVE_MODE     = 0x21  # 波形模式: 0=关, 1=正弦, 2=方波, 3=三角
CMD_WAVE_FREQ     = 0x22  # 频率(mHz)
CMD_WAVE_OFFSET   = 0x23  # DC偏置(mV)
CMD_WAVE_AMP      = 0x24  # 幅度(mV, x1000)

# ── 协议工具函数 ──

def u32_to_big(val):
    """uint32 → 大端4字节"""
    return struct.pack('>I', val & 0xFFFFFFFF)

def calc_checksum(data):
    """校验和 = 0xFF - sum(bytes[1..len-2])"""
    s = sum(data[1:-2]) & 0xFF
    return (0xFF - s) & 0xFF

def make_packet(cmd, data1=0, data2=0, data3=0):
    """构造36字节发送帧"""
    pkt = bytearray(RX_FRAME_LEN)
    pkt[0] = FRAME_HEADER
    pkt[5] = cmd
    pkt[10:14] = u32_to_big(data1)
    pkt[14:18] = u32_to_big(data2)
    pkt[18:22] = u32_to_big(data3)
    pkt[34] = calc_checksum(pkt)
    pkt[35] = FRAME_TAIL
    return bytes(pkt)

# ── 回复解析 ──

def parse_collect_reply(buf):
    """解析82字节采集回复 (CMD_LOOP_CHECK)"""
    if len(buf) < TX_COL_LEN:
        return None
    volt = struct.unpack('>I', buf[20:24])[0]   # mV
    curr = struct.unpack('>I', buf[24:28])[0]   # x1000
    temp = struct.unpack('>I', buf[48:52])[0]   # x1000
    return {
        'voltage': volt / 1000.0,
        'dac': curr / 1000.0,
        'temperature': temp / 1000.0,
    }

def parse_settings_reply(buf):
    """解析86字节设置回复 (CMD_DEVICE_CHECK)"""
    if len(buf) < TX_SET_LEN:
        return None
    curr_set = struct.unpack('>I', buf[8:12])[0]   # x1000
    bias_set = struct.unpack('>I', buf[24:28])[0]  # x10000
    pid_p = struct.unpack('>I', buf[40:44])[0]     # x1000
    pid_i = struct.unpack('>I', buf[44:48])[0]     # x1000
    pid_d = struct.unpack('>I', buf[48:52])[0]     # x1000
    ff    = struct.unpack('>I', buf[52:56])[0]     # x1000
    wfreq = struct.unpack('>I', buf[56:60])[0]     # mHz
    woff  = struct.unpack('>I', buf[60:64])[0]     # mV
    wmode = struct.unpack('>I', buf[64:68])[0]     # 0/1/2/3
    wave  = struct.unpack('>I', buf[72:76])[0]     # x1000
    pd    = struct.unpack('>I', buf[76:80])[0]     # x10
    return {
        'current_set': curr_set / 1000.0,
        'bias_set': bias_set / 10000.0,
        'pid_p': pid_p / 1000.0,
        'pid_i': pid_i / 1000.0,
        'pid_d': pid_d / 1000.0,
        'ff': ff / 1000.0,
        'wave_freq': wfreq,
        'wave_off': woff / 1000.0,
        'wave_mode': wmode,
        'wave_amp': wave / 1000.0,
        'pd_sample': pd / 10.0,
    }

# ── 上位机类 ──

class STM32Tester:
    def __init__(self, port, baud=115200, timeout=0.5):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.rx_buf = bytearray()
        self._lock = threading.Lock()
        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()
        print(f"[✓] 串口已打开: {port} @ {baud}")

    def _rx_loop(self):
        """后台接收线程: 按长度匹配回复帧"""
        while self._running:
            try:
                data = self.ser.read(256)
                if not data:
                    continue
                with self._lock:
                    self.rx_buf.extend(data)
                    self._try_dispatch()
            except Exception:
                break

    def _try_dispatch(self):
        """从 rx_buf 中提取完整帧"""
        while len(self.rx_buf) >= TX_STD_LEN:
            # 找帧头
            head_idx = -1
            for i in range(len(self.rx_buf)):
                if self.rx_buf[i] == FRAME_HEADER:
                    head_idx = i
                    break
            if head_idx < 0:
                self.rx_buf.clear()
                return

            # 丢弃帧头前的垃圾
            if head_idx > 0:
                self.rx_buf = self.rx_buf[head_idx:]
                continue

            # 判断帧类型 (根据帧长)
            length = len(self.rx_buf)

            # 检查尾部
            for (flen, name) in [(TX_SET_LEN, 'SETTINGS'),
                                  (TX_COL_LEN, 'COLLECT'),
                                  (TX_STD_LEN, 'STD_REPLY')]:
                if length >= flen and self.rx_buf[flen-1] == FRAME_TAIL:
                    frame = self.rx_buf[:flen]
                    self.rx_buf = self.rx_buf[flen:]

                    # 校验和验证
                    cal = calc_checksum(frame)
                    if cal != frame[flen-2]:
                        print(f"[!] {name} 校验和失败 (计算=0x{cal:02X}, 收到=0x{frame[flen-2]:02X})")
                        continue

                    cmd = frame[5]
                    status = frame[8] if flen == TX_STD_LEN else -1

                    if flen == TX_STD_LEN:
                        self._on_std_reply(cmd, status)
                    elif flen == TX_COL_LEN:
                        self._on_collect_reply(frame)
                    elif flen == TX_SET_LEN:
                        self._on_settings_reply(frame)
                    break
            else:
                break  # 没有完整帧

    def _on_std_reply(self, cmd, status):
        print(f"[<] 标准回复 | CMD=0x{cmd:02X} | Status=0x{status:02X}")

    def _on_collect_reply(self, frame):
        r = parse_collect_reply(frame)
        if r:
            diff = r['voltage'] - r['dac']
            print(f"[<] 采集数据 | ADC={r['voltage']:.4f}V | "
                  f"DAC={r['dac']:.4f}V | Δ={diff:+.4f}V | 温度={r['temperature']:.3f}°C")

    def _on_settings_reply(self, frame):
        r = parse_settings_reply(frame)
        if r:
            wnames = {0:'OFF', 1:'SINE', 2:'SQUARE', 3:'TRI'}
            print(f"[<] 设备参数 | P={r['pid_p']:.3f} I={r['pid_i']:.3f} D={r['pid_d']:.3f} | "
                  f"FF={r['ff']:.3f}V | 目标={r['bias_set']:.3f}V | "
                  f"波={wnames.get(r['wave_mode'],'?' )} {r['wave_freq']}mHz "
                  f"幅度={r['wave_amp']:.3f}V 偏置={r['wave_off']:.3f}V")

    def send(self, cmd, data1=0, data2=0, data3=0, label=""):
        pkt = make_packet(cmd, data1, data2, data3)
        self.ser.write(pkt)
        label = label or f"CMD=0x{cmd:02X}"
        print(f"[→] {label} | data1={data1} data2={data2} data3={data3}")

    def close(self):
        self._running = False
        time.sleep(0.1)
        self.ser.close()

    # ── 高级命令 ──

    def device_check(self):
        self.send(CMD_DEVICE_CHECK, label="DEVICE_CHECK")

    def loop_check(self):
        self.send(CMD_LOOP_CHECK, label="LOOP_CHECK")

    def ld_enable(self):
        self.send(CMD_LD_ENABLE, label="LD_ENABLE")

    def ld_disable(self):
        self.send(CMD_LD_DISABLE, label="LD_DISABLE")

    def bias_enable(self):
        self.send(CMD_BIAS_V1_ENABLE, label="BIAS_V1_ENABLE")

    def set_current(self, curr_ma):
        """设置LD电流, curr_ma单位mA, 协议单位x100"""
        self.send(CMD_CURRENT_SET, int(curr_ma * 100), label=f"CURRENT_SET={curr_ma}mA")

    def set_voltage(self, volt_mv):
        """设置目标电压, volt_mv单位mV"""
        self.send(CMD_BIAS_V1_SET, int(volt_mv), label=f"BIAS_V1_SET={volt_mv}mV")

    def set_pid(self, p, i, d):
        """设置PID参数 (浮点数)"""
        self.send(CMD_PID_SET,
                  int(p * 1000),
                  int(i * 1000),
                  int(d * 1000),
                  label=f"PID_SET P={p:.3f} I={i:.3f} D={d:.3f}")

    def set_ff(self, volt_mv):
        """设置前馈偏置(mV, x1000)"""
        self.send(CMD_FF_SET, int(volt_mv),
                  label=f"FF_SET={volt_mv}mV")

    def set_wave_mode(self, mode):
        """波形模式: 0=关(PID), 1=正弦, 2=方波, 3=三角"""
        names = {0:'OFF(PID)', 1:'SINE', 2:'SQUARE', 3:'TRIANGLE'}
        self.send(CMD_WAVE_MODE, mode,
                  label=f"WAVE_MODE={names.get(mode,mode)}")

    def set_wave_freq(self, freq_mhz):
        """波形频率(mHz, 如 1000=1Hz)"""
        self.send(CMD_WAVE_FREQ, freq_mhz,
                  label=f"WAVE_FREQ={freq_mhz}mHz")

    def set_wave_offset(self, mv):
        """波形DC偏置(mV)"""
        self.send(CMD_WAVE_OFFSET, mv,
                  label=f"WAVE_OFFSET={mv}mV")

    def set_wave_amp(self, amp_mv):
        """波形幅度(mV, x1000)"""
        self.send(CMD_WAVE_AMP, int(amp_mv),
                  label=f"WAVE_AMP={amp_mv}mV")

    def set_dac_direct(self, volt_mv):
        """开环直接设DAC电压, 0=恢复PID"""
        self.send(CMD_DAC_DIRECT, int(volt_mv),
                  label=f"DAC_DIRECT={'PID' if volt_mv==0 else f'{volt_mv}mV'}")


def list_ports():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("未检测到串口设备")
        return []
    for p in ports:
        print(f"  {p.device} — {p.description}")
    return ports


def interactive(tester):
    """交互模式"""
    print("\n" + "="*60)
    print("STM32 上位机测试 — 交互模式")
    print("="*60)
    print("可用命令:")
    print("  1      设备查询 (DEVICE_CHECK)")
    print("  2      使能LD")
    print("  3      禁用LD")
    print("  4      使能偏压")
    print("  5      循环查询 (LOOP_CHECK ×5)")
    print("  6      设定目标电压  (例: 6 1500 → 1.500V)")
    print("  7      设置PID       (例: 7 0.5 0.2 0.0)")
    print("  8      设置电流      (例: 8 100 → 100mA)")
    print("  9      开环直接设DAC (例: 9 2000 → 2.0V, 9 0 → 恢复PID)")
    print("  0      前馈偏置      (例: 0 -150 → -0.150V)")
    print("  w      波形模式      (例: w 1 = 正弦, w 0 = 关)")
    print("  f      频率          (例: f 500 = 0.5Hz)")
    print("  o      DC偏置        (例: o 1000 = 1.0V)")
    print("  a      幅度          (例: a 500 = ±0.5V)")
    print("  p      实时绘图      (采集15秒绘ADC/DAC曲线)")
    print("  q      退出")
    print("-"*60)
    try:
        while True:
            cmd = input(">> ").strip()
            if not cmd:
                continue
            parts = cmd.split()
            action = parts[0].lower()
            args = parts[1:]

            if action == 'q':
                break
            elif action == '1':
                tester.device_check()
            elif action == '2':
                tester.ld_enable()
            elif action == '3':
                tester.ld_disable()
            elif action == '4':
                tester.bias_enable()
            elif action == '5':
                for i in range(5):
                    tester.loop_check()
                    time.sleep(0.2)
            elif action == '6' and len(args) >= 1:
                tester.set_voltage(int(float(args[0])))
            elif action == '7' and len(args) >= 3:
                tester.set_pid(float(args[0]), float(args[1]), float(args[2]))
            elif action == '8' and len(args) >= 1:
                tester.set_current(float(args[0]))
            elif action == '9' and len(args) >= 1:
                tester.set_dac_direct(int(float(args[0])))
            elif action == '0' and len(args) >= 1:
                tester.set_ff(int(float(args[0])))
            elif action == 'w' and len(args) >= 1:
                tester.set_wave_mode(int(args[0]))
            elif action == 'f' and len(args) >= 1:
                tester.set_wave_freq(int(float(args[0])))
            elif action == 'o' and len(args) >= 1:
                tester.set_wave_offset(int(float(args[0])))
            elif action == 'a' and len(args) >= 1:
                tester.set_wave_amp(int(float(args[0])))
            elif action == 'p':
                plot_wave(tester, duration=10, interval=0.02)
            else:
                print("未知命令或参数不足")
    except KeyboardInterrupt:
        print()


def quick_test(tester):
    """一键快速验证所有功能"""
    print("\n" + "="*60)
    print("快速测试 (全部命令自动化验证)")
    print("="*60)

    # 1. 设备查询
    print("\n[测试1] DEVICE_CHECK (0x01)")
    tester.device_check()
    time.sleep(0.3)

    # 2. 使能 LD
    print("\n[测试2] LD_ENABLE (0x02)")
    tester.ld_enable()
    time.sleep(0.2)

    # 3. 禁用 LD
    print("\n[测试3] LD_DISABLE (0x03)")
    tester.ld_disable()
    time.sleep(0.2)

    # 4. 使能偏压
    print("\n[测试4] BIAS_V1_ENABLE (0x06)")
    tester.bias_enable()
    time.sleep(0.2)

    # 5. 设置电流
    print("\n[测试5] CURRENT_SET (0x11) = 50mA")
    tester.set_current(50.0)
    time.sleep(0.2)

    # 6. 设目标电压
    print("\n[测试6] BIAS_V1_SET (0x13) = 1.500V")
    tester.set_voltage(1500)
    time.sleep(0.2)

    # 7. 循环查询
    print("\n[测试7] LOOP_CHECK (0x18) ×3")
    for i in range(3):
        tester.loop_check()
        time.sleep(0.2)

    # 8. PID设置
    print("\n[测试8] PID_SET (0x1C) P=0.8 I=0.1 D=0.01")
    tester.set_pid(0.8, 0.1, 0.01)
    time.sleep(0.2)

    print("\n" + "="*60)
    print("快速测试完成！")
    print("="*60)


def continuous_monitor(tester, interval=0.5):
    """连续采集模式"""
    print(f"\n连续采集模式 (间隔{interval}s, Ctrl+C停止)")
    print(f"{'ADC(V)':>10s}  {'DAC(V)':>10s}  {'Δ(V)':>8s}  {'温度(°C)':>8s}")
    print("-"*45)
    try:
        while True:
            tester.loop_check()
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\n采集停止")


def wave_monitor(tester, interval=0.2):
    """波形监视模式: 连续采集, 显示电压变化"""
    print(f"\n波形监视模式 (间隔{interval}s, Ctrl+C停止)")
    print(f"{'ADC(V)':>10s}  {'DAC(V)':>10s}  {'Δ(V)':>8s}")
    print("-"*35)
    try:
        while True:
            tester.loop_check()
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\n监视停止")


def plot_wave(tester, duration=10, interval=0.01):
    """实时绘图模式: 用matplotlib绘制ADC/DAC波形曲线"""
    import matplotlib.pyplot as plt
    from collections import deque

    print(f"\n实时绘图模式 (采集{duration}秒, 间隔{interval}s)")
    print("等待数据中...")

    points = int(duration / interval)
    t0 = None  # 首帧时间戳
    t_data = deque(maxlen=points)
    adc_data = deque(maxlen=points)
    dac_data = deque(maxlen=points)

    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 5))
    adc_line, = ax.plot([], [], 'b-', label='ADC (测量)', linewidth=1.5)
    dac_line, = ax.plot([], [], 'r-', label='DAC (输出)', linewidth=1.5, alpha=0.7)
    ax.set_ylim(0, 5)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('电压 (V)')
    ax.set_title('ADC / DAC 实时波形')
    ax.legend()
    ax.grid(True, alpha=0.3)

    data_lock = threading.Lock()
    plot_t0 = [None]  # list hack for Python 2/3 compat in nested scope

    # hook into tester's rx handler
    original_handler = tester._on_collect_reply

    def collect_hook(frame):
        r = parse_collect_reply(frame)
        if r:
            with data_lock:
                now = time.time()
                if plot_t0[0] is None:
                    plot_t0[0] = now
                t_data.append(now - plot_t0[0])
                adc_data.append(r['voltage'])
                dac_data.append(r['dac'])

    tester._on_collect_reply = collect_hook

    try:
        start_time = time.time()
        while time.time() - start_time < duration:
            tester.loop_check()
            time.sleep(interval)

            with data_lock:
                if len(adc_data) > 1:
                    adc_line.set_data(list(t_data), list(adc_data))
                    dac_line.set_data(list(t_data), list(dac_data))
                    ax.set_xlim(max(0, t_data[-1] - duration), max(duration, t_data[-1] + 1))
                    ax.relim()
                    ax.autoscale_view(scalex=False)
            plt.pause(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        tester._on_collect_reply = original_handler
        plt.ioff()
        plt.show()

    print(f"\n绘图完成, 共采集 {len(adc_data)} 个点")


def calibrate(tester):
    """自动标定: 从0.1V到3.3V步进0.2V, 每步读ADC"""
    import csv
    from datetime import datetime

    print("\n" + "="*60)
    print("自动标定模式 — DAC→ADC 映射")
    print("="*60)
    print(f"{'DAC目标(V)':>12s}  {'ADC读数(V)':>12s}  {'偏差(V)':>10s}  {'偏差(%)':>8s}")
    print("-"*55)

    # 暂停后台接收线程避免干扰
    tester._running = False
    time.sleep(0.2)

    results = []
    steps = [round(x * 0.2, 1) for x in range(0, 17)]
    steps.append(3.3)

    for dac_v in steps:
        dac_mv = int(dac_v * 1000)

        # 发开环DAC命令
        pkt = make_packet(CMD_DAC_DIRECT, dac_mv)
        tester.ser.write(pkt)
        time.sleep(0.5)  # 等DAC和ADC稳定

        # 连续发LOOP_CHECK并读回复, 取最后一个
        last_volt = None
        for _ in range(3):
            tester.ser.write(make_packet(CMD_LOOP_CHECK))
            time.sleep(0.15)
            # 读回复
            data = tester.ser.read(256)
            # 找82字节的collect帧
            idx = data.find(b'\x68')
            while idx >= 0:
                if len(data) - idx >= TX_COL_LEN and data[idx + TX_COL_LEN - 1] == FRAME_TAIL:
                    frame = data[idx:idx + TX_COL_LEN]
                    r = parse_collect_reply(frame)
                    if r:
                        last_volt = r['voltage']
                    break
                idx = data.find(b'\x68', idx + 1)

        if last_volt is not None:
            err_v = last_volt - dac_v
            err_pct = (err_v / dac_v * 100) if dac_v > 0 else 0
            results.append((dac_v, last_volt, err_v, err_pct))
            print(f"  {dac_v:>8.1f}V     {last_volt:>8.4f}V     {err_v:>+7.4f}V  {err_pct:>+7.2f}%")
        else:
            results.append((dac_v, 0, 0, 0))
            print(f"  {dac_v:>8.1f}V     {'N/A':>>10s}")

    # 恢复后台线程
    tester._running = True
    tester._rx_thread = threading.Thread(target=tester._rx_loop, daemon=True)
    tester._rx_thread.start()

    # 恢复PID
    tester.ser.write(make_packet(CMD_DAC_DIRECT, 0))
    time.sleep(0.1)

    print("-"*55)
    print("标定完成! 结果已保存到 calibration.csv")

    # 保存CSV
    with open('calibration.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['DAC_V', 'ADC_V', 'ERROR_V', 'ERROR_PCT'])
        w.writerows(results)


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='STM32 上位机测试工具')
    parser.add_argument('port', nargs='?', help='串口号, 如 COM3')
    parser.add_argument('-b', '--baud', type=int, default=115200, help='波特率 (默认115200)')
    parser.add_argument('-m', '--mode', choices=['interactive', 'quick', 'monitor', 'calibrate', 'wave', 'plot'],
                        default='interactive', help='测试模式 (默认interactive)')
    parser.add_argument('-i', '--interval', type=float, default=0.5, help='monitor模式采样间隔(秒)')
    args = parser.parse_args()

    # 自动查找串口
    port = args.port
    if not port:
        print("可用串口:")
        ports = list_ports()
        if not ports:
            sys.exit(1)
        port = ports[0].device
        print(f"\n自动选择: {port}")
        print("指定串口: python test_host.py COM3")
        print("查看帮助: python test_host.py -h")
        print()

    tester = STM32Tester(port, args.baud)
    time.sleep(0.3)  # 等待串口稳定

    try:
        if args.mode == 'quick':
            quick_test(tester)
        elif args.mode == 'monitor':
            continuous_monitor(tester, args.interval)
        elif args.mode == 'calibrate':
            calibrate(tester)
        elif args.mode == 'wave':
            wave_monitor(tester, args.interval)
        elif args.mode == 'plot':
            plot_wave(tester, duration=15, interval=args.interval)
        else:
            interactive(tester)
    finally:
        tester.close()
        print("串口已关闭")