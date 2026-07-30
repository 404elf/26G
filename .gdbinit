python
import matplotlib.pyplot as plt
import gdb

ADC_REF_V = 3.3
ADC_RES = 4096

# ==================== 通用绘图 ====================
class PlotBuffer(gdb.Command):
    """plot_buf <数组名> <长度> — 通用数组绘图"""
    def __init__(self):
        super(PlotBuffer, self).__init__("plot_buf", gdb.COMMAND_USER)

    def invoke(self, arg, from_tty):
        args = gdb.string_to_argv(arg)
        buf_name = args[0]
        length = int(args[1])
        
        frame = gdb.selected_frame()
        val = gdb.parse_and_eval(buf_name)
        data = [int(val[i]) for i in range(length)]
        
        plt.figure("MCU Memory Plot")
        plt.plot(data, 'b-', label=buf_name)
        plt.grid(True)
        plt.legend()
        plt.show()

PlotBuffer()

# ==================== ADC 专用拆包 + 绘图 ====================
class AdcPlot(gdb.Command):
    """adc_plot [half] — 读取 adc_dma_buf，拆包并绘制波形

    可选参数：
      half   — 只读前半帧 2048 word，DMA 最早写完、最稳定的那半
    使用例：
      (gdb) adc_plot      读取全部 4096 word → 8192 采样点
      (gdb) adc_plot half  只读前半帧 2048 word → 4096 采样点
    """
    def __init__(self):
        super(AdcPlot, self).__init__("adc_plot", gdb.COMMAND_USER)

    def invoke(self, arg, from_tty):
        args = gdb.string_to_argv(arg)
        read_half = (len(args) > 0 and args[0] == "half")

        # 从 MCU 读取 adc_dma_buf
        val = gdb.parse_and_eval("adc_dma_buf")

        if read_half:
            n_words = 2048
            offset = 0               # 前半帧：DMA 最先写完的区，最稳定
            title_suffix = "（前半帧）"
        else:
            n_words = 4096
            offset = 0
            title_suffix = "（全缓冲）"

        raw = [int(val[offset + i]) for i in range(n_words)]

        # === 拆包：每个 32-bit word → 两个 12-bit 采样 ===
        samples = []
        for w in raw:
            s0 = (w >> 0) & 0xFFF
            s1 = (w >> 16) & 0xFFF
            samples.append(s0)
            samples.append(s1)

        n = len(samples)
        voltages = [s * ADC_REF_V / ADC_RES for s in samples]

        # === 统计 ===
        s_min = min(samples)
        s_max = max(samples)
        s_avg = sum(samples) / n
        v_min = min(voltages)
        v_max = max(voltages)
        v_avg = sum(voltages) / n

        print(f"ADC DMA 拆包结果{title_suffix}:")
        print(f"  {n_words} word → {n} 采样点")
        print(f"  码值: min={s_min}  max={s_max}  avg={s_avg:.0f}  Vpp={s_max - s_min}")
        print(f"  电压: min={v_min:.3f}V  max={v_max:.3f}V  avg={v_avg:.3f}V  Vpp={v_max - v_min:.3f}V")

        # === 绘图 ===
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 6), sharex=True)
        x = list(range(n))

        ax1.plot(x, samples, linewidth=0.5)
        ax1.axhline(y=s_avg, color='r', linestyle='--', alpha=0.5, label=f"均值={s_avg:.0f}")
        ax1.set_ylabel("ADC 码值")
        ax1.set_title(f"ADC DMA 采样波形{title_suffix}  ({n} 点)")
        ax1.legend(fontsize=9)
        ax1.grid(True, alpha=0.3)

        ax2.plot(x, voltages, linewidth=0.5, color='C1')
        ax2.axhline(y=v_avg, color='r', linestyle='--', alpha=0.5, label=f"均值={v_avg:.3f}V")
        if v_max - v_min > 0.01:
            ax2.axhline(y=v_max, color='g', linestyle=':', alpha=0.5, label=f"峰值={v_max:.3f}V")
            ax2.axhline(y=v_min, color='g', linestyle=':', alpha=0.5, label=f"谷值={v_min:.3f}V")
        ax2.set_xlabel("采样序号")
        ax2.set_ylabel("电压 (V)")
        ax2.legend(fontsize=9)
        ax2.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show()

AdcPlot()
end
# 使用实例：
#   (gdb) plot_buf DAC_Buffer 1024     — 通用数组绘图
#   (gdb) adc_plot                     — 读取全部 adc_dma_buf，拆包 + 绘图
#   (gdb) adc_plot half                — 只读前半帧（DMA 最早写完、最稳定的区）
