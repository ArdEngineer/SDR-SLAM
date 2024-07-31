# Euroc MAV数据集使用教程

## 把MAV数据包转换成rosbag包

把MH数据包解压出来，一般解压出来的是一个名为mav0的文件夹

假设这个文件夹(mav0)在文件夹/home/Desktop/下面，那么就在Desktop下打开终端

然后在project_resource下面有一个bagcreater.py的python脚本，这个脚本就用于将

mav数据转换成rosbag包，运行的指令如下

```
python3 bagcreater.py --folder mav0 --output-bag <bag_name>.bag
```

<bag_name>可以自己取其他名字

然后你就会在当前文件下得到<bag_name>.bag这个rosbag包，这个就是我们需要的东西

如何运行rosbag包？参见教程tutorials里面的

vins-mono/vins-fusion + euroc数据集运行教程，这里不多赘述