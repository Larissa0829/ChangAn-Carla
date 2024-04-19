'''
该传感器集成了 CARLA 中的 C++ 责任敏感安全库。默认情况下，它在 CARLA 中处于禁用状态，必须显式构建才能使用。

RSS 传感器计算车辆的 RSS 状态，并将当前 RSS 响应作为传感器数据检索。卡拉。RssRestrictor 将使用此数据来调整 carla。VehicleControl，然后再将其应用于车辆。

这些控制器可以由自动驾驶堆栈或用户输入生成。例如，下面有一个代码片段，其中用户输入在必要时使用 RSS 进行修改。PythonAPI/examples/rss/manual_control_rss.py

1. 检查 RssSensor 是否生成包含限制的有效响应。2. 收集车辆的当前动态和车辆物理。3. 使用 RssSensor 的响应以及车辆的当前动力学和物理特性对车辆控制施加限制。
'''