python
import matplotlib.pyplot as plt
import gdb

class PlotBuffer(gdb.Command):
    """自定义 GDB 命令：plot_buf <数组名> <长度>"""
    def __init__(self):
        super(PlotBuffer, self).__init__("plot_buf", gdb.COMMAND_USER)

    def invoke(self, arg, from_tty):
        args = gdb.string_to_argv(arg)
        buf_name = args[0]
        length = int(args[1])
        
        # 直接从 MCU 内存读取数组数据
        frame = gdb.selected_frame()
        val = gdb.parse_and_eval(buf_name)
        
        data = [int(val[i]) for i in range(length)]
        
        # 用 Matplotlib 弹窗画图
        plt.figure("MCU Memory Plot")
        plt.plot(data, 'b-', label=buf_name)
        plt.grid(True)
        plt.legend()
        plt.show()

PlotBuffer()
end
#使用实例：(gdb) plot_buf DAC_Buffer 1024
