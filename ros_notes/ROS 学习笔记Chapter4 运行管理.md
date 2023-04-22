# ROS 运行管理

本章主要内容介绍在ROS中上述问题的解决策略(见本章目录)，预期达成学习目标也与上述问题对应：

- 掌握元功能包使用语法；
- 掌握launch文件的使用语法；
- 理解什么是ROS工作空间覆盖，以及存在什么安全隐患；
- 掌握节点名称重名时的处理方式；
- 掌握话题名称重名时的处理方式；
- 掌握参数名称重名时的处理方式；
- 能够实现ROS分布式通信。

## 1. 元功能包

### 概念

MetaPackage是Linux的一个文件管理系统的概念。是ROS中的一个虚包，里面没有实质性的内容，但是它依赖了其他的软件包，通过这种方法可以把其他包组合起来，我们可以认为它是一本书的目录索引，告诉我们这个包集合中有哪些子包，并且该去哪里下载。

例如：

- sudo apt install ros-noetic-desktop-full 命令安装ros时就使用了元功能包，该元功能包依赖于ROS中的其他一些功能包，安装该包时会一并安装依赖。

还有一些常见的MetaPackage：navigation moveit! turtlebot3 ....

### 作用

方便用户的安装，我们只需要这一个包就可以把其他相关的软件包组织到一起安装了。

### 实现

**首先:**新建一个功能包

**然后:**修改**package.xml** ,内容如下:

```xml
 <exec_depend>被集成的功能包</exec_depend>
 .....
 <export>
   <metapackage />
 </export>
Copy
```

**最后:**修改 CMakeLists.txt,内容如下:

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(demo) 注意！这里的demo是元功能包的包名
find_package(catkin REQUIRED)
catkin_metapackage()
Copy
```

PS:CMakeLists.txt 中不可以有换行。

## 2. Launch文件应用

### 2.1 标签

`<launch>`标签是所有 launch 文件的根标签，充当其他标签的容器

#### 1.属性

- `deprecated = "弃用声明"`

  告知用户当前 launch 文件已经弃用

  例如：

  ```
  <launch deprecated="本文件已经弃用">
  </launch>
  ```

#### 2.子级标签

所有其它标签都是launch的子级

### 2.2 launch文件标签之node

`<node>`标签用于指定 ROS 节点，是最常见的标签，需要注意的是: roslaunch 命令不能保证按照 node 的声明顺序来启动节点(节点的启动是多进程的)

#### 1.属性

- pkg="包名"

  节点所属的包

- type="nodeType"

  节点类型(与之相同名称的可执行文件)

- name="nodeName"

  节点名称(在 ROS 网络拓扑中节点的名称)

- args="xxx xxx xxx" (可选)

  将参数传递给节点

- machine="机器名"

  在指定机器上启动节点

- respawn="true | false" (可选)

  如果节点退出，是否自动重启

- respawn_delay=" N" (可选)

  如果 respawn 为 true, 那么延迟 N 秒后启动节点

- required="true | false" (可选)

  该节点是否必须，如果为 true,那么如果该节点退出，将杀死整个 roslaunch

- ns="xxx" (可选)

  在指定命名空间 xxx 中启动节点

- clear_params="true | false" (可选)

  在启动前，删除节点的私有空间的所有参数

- output="log | screen" (可选)

  日志发送目标，可以设置为 log 日志文件，或 screen 屏幕,默认是 log

#### 2.子级标签

- env 环境变量设置
- remap 重映射节点名称
- rosparam 参数设置
- param 参数设置

### 2.3 launch文件标签之include

`include`标签用于将另一个 xml 格式的 launch 文件导入到当前文件

#### 1.属性

- file="$(find 包名)/xxx/xxx.launch"

  要包含的文件路径

- ns="xxx" (可选)

  在指定命名空间导入文件

例如：

```
<launch>
    <include file="$(find control_turtle)/launch/start_turtle.launch">
</launch>
```

#### 2.子级标签

- env 环境变量设置
- arg 将参数传递给被包含的文件

### 2.4 launch文件标签之remap

用于话题重命名

#### 1.属性

- from="xxx"

  原始话题名称

- to="yyy"

  目标名称

  例如：

  ```
  <launch>
     <node pkg=....>
        <remap from="xxx" to="yyy" />
     </node>
  </launch>
  ```

  

#### 2.子级标签

- 无

### 2.5 launch文件标签之param

`<param>`标签主要用于在参数服务器上设置参数，参数源可以在标签中通过 value 指定，也可以通过外部文件加载，在`<node>`标签中时，相当于私有命名空间。

#### 1.属性

- name="命名空间/参数名"

  参数名称，可以包含命名空间

- value="xxx" (可选)

  定义参数值，如果此处省略，必须指定外部文件作为参数源

- type="str | int | double | bool | yaml" (可选)

  指定参数类型，如果未指定，roslaunch 会尝试确定参数类型，规则如下:

  - 如果包含 '.' 的数字解析未浮点型，否则为整型
  - "true" 和 "false" 是 bool 值(不区分大小写)
  - 其他是字符串

#### 2.子级标签

- 无

### 2.6 launch文件标签之rosparam

`<rosparam>`标签可以从 YAML 文件导入参数，或将参数导出到 YAML 文件，也可以用来删除参数，`<rosparam>`标签在`<node>`标签中时被视为私有。

#### 1.属性

- command="load | dump | delete" (可选，默认 load) dump为导出参数

  加载、导出或删除参数

- file="$(find 功能包名)/launch/param.yaml"    一般将yaml文件放到该功能包的launch文件夹中

  加载或导出到的 yaml 文件

- param="参数名称"

- ns="命名空间" (可选)

#### 2.子级标签

- 无

### 2.7 launch文件标签之group

<group>标签可以对节点分组，具有 ns 属性，可以让节点归属某个命名空间。一个命名空间对应一个海龟窗口

#### 1.属性

ns="名称空间" (可选)

clear_params="true | false" (可选)

启动前，是否删除组名称空间的所有参数(慎用....此功能危险)

#### 2.子级标签

除了launch 标签外的其他标签

### 2.8 launch文件标签之arg

`<arg>`标签是用于动态传参，类似于函数的参数，可以增强launch文件的灵活性

#### 1.属性

- name="参数名称"

- default="默认值" (可选)

- value="数值" (可选)

  不可以与 default 并存

- doc="描述"

  参数说明

#### 2.子级标签

- 无

#### 3.示例

- launch文件传参语法实现,hello.lcaunch

  ```xml
  <launch>
      <arg name="xxx" />
      <param name="param" value="$(arg xxx)" />
  </launch>
  Copy
  ```

- 命令行调用launch传参

  ```
  roslaunch hello.launch xxx:=值
  ```

## 3 ROS工作空间覆盖

所谓工作空间覆盖，是指不同工作空间中，存在重名的功能包的情形。

> ROS 开发中，会自定义工作空间且自定义工作空间可以同时存在多个，可能会出现一种情况: 虽然特定工作空间内的功能包不能重名，但是自定义工作空间的功能包与内置的功能包可以重名或者不同的自定义的工作空间中也可以出现重名的功能包，那么调用该名称功能包时，会调用哪一个呢？比如：自定义工作空间A存在功能包 turtlesim，自定义工作空间B也存在功能包 turtlesim，当然系统内置空间也存在turtlesim，如果调用turtlesim包，会调用哪个工作空间中的呢？

在根目录./bashrc中，如果source 多个工作空间，如：

```
source /home/用户/路径/工作空间A/devel/setup.bash
source /home/用户/路径/工作空间B/devel/setup.bash
```

则在执行相同名称的功能包时，会执行工作空间B的功能包，因为B在A后面

BUG 说明:

> 当在 .bashrc 文件中 source 多个工作空间后，可能出现的情况，在 ROS PACKAGE PATH 中只包含两个工作空间，可以删除自定义工作空间的 build 与 devel 目录，重新 catkin_make，然后重新载入 .bashrc 文件，问题解决。

**小技巧**：可以建立一个单独的工作空间C管理其他工作空间

在空间C/devel/_setup_util.py中，修改CMAKE_PREFIX_PATH，将其他工作空间的devel目录添加进去，如：

```makefile
if not args.local:
environment at generation time
            CMAKE_PREFIX_PATH = '/opt/ros/melodic;/home/chengyangkj/catkin_qt/devel'.split(';')
        else:
            # don't consider any other prefix path than this one
            CMAKE_PREFIX_PATH = []
```

然后在./bashrc里面只添加空间C就可以啦，这样其他工作空间也会出现在环境变量里

## 4 ROS节点名称重名

> 场景:ROS 中创建的节点是有名称的，C++初始化节点时通过API:`ros::init(argc,argv,"xxxx");`来定义节点名称，在Python中初始化节点则通过 `rospy.init_node("yyyy")` 来定义节点名称。在ROS的网络拓扑中，是不可以出现重名的节点的，因为假设可以重名存在，那么调用时会产生混淆，这也就意味着，不可以启动重名节点或者同一个节点启动多次，的确，在ROS中如果启动重名节点的话，之前已经存在的节点会被直接关闭，但是如果有这种需求的话，怎么优化呢？

在ROS中给出的解决策略是使用命名空间或名称重映射。比如使用如下命令：

- rosrun 命令

  通过命名空间来实现

  语法: rosrun 包名 节点名 __ns:=新名称

  ```
  rosrun turtlesim turtlesim_node __ns:=/xxx
  rosrun turtlesim turtlesim_node __ns:=/yyy
  ```

  两个节点都可以运行

  `rosnode list`查看节点信息,显示结果:

  ```
  /xxx/turtlesim
  /yyy/turtlesim
  ```

  通过重映射来实现，给节点起别名

  语法: rosrun 包名 节点名 __name:=新名称

- launch 文件

  ```xml
  <launch>
  
      <node pkg="turtlesim" type="turtlesim_node" name="t1" />
      <node pkg="turtlesim" type="turtlesim_node" name="t2" />
      <node pkg="turtlesim" type="turtlesim_node" name="t1" ns="hello"/>
  
  </launch>
  ```

  `rosnode list`查看节点信息,显示结果:

  ```
  /t1
  /t2
  /t1/hello
  ```

- 编码实现

  1.C++ 实现:重映射

  1.1名称别名设置

  核心代码:`ros::init(argc,argv,"zhangsan",ros::init_options::AnonymousName);`

  1.2执行

  会在名称后面添加时间戳。

  2.C++ 实现:命名空间

  2.1命名空间设置

  核心代码

  ```
    std::map<std::string, std::string> map;
    map["__ns"] = "xxxx";
    ros::init(map,"wangqiang");
  Copy
  ```

  2.2执行

  节点名称设置了命名空间。

## 5 ROS话题名称设置

在ROS中节点名称可能出现重名的情况，同理话题名称也可能重名。

> 在 ROS 中节点终端，不同的节点之间通信都依赖于话题，话题名称也可能出现重复的情况，这种情况下，系统虽然不会抛出异常，但是可能导致订阅的消息非预期的，从而导致节点运行异常。这种情况下需要将两个节点的话题名称由相同修改为不同。
>
> 又或者，两个节点是可以通信的，两个节点之间使用了相同的消息类型，但是由于，话题名称不同，导致通信失败。这种情况下需要将两个节点的话题名称由不同修改为相同。

在实际应用中，按照逻辑，有些时候可能需要将相同的话题名称设置为不同，也有可能将不同的话题名设置为相同。在ROS中给出的解决策略与节点名称重命类似，也是使用名称重映射或为名称添加前缀。根据前缀不同，有全局、相对、和私有三种类型之分。

- 全局(参数名称直接参考ROS系统，与节点命名空间平级)
- 相对(参数名称参考的是节点的命名空间，与节点名称平级)
- 私有(参数名称参考节点名称，是节点名称的子级)

**rosrun名称重映射语法: rorun 包名 节点名 话题名:=新话题名称**

将乌龟显示节点的话题设置为 `/cmd_vel`

启动键盘控制节点:`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

启动乌龟显示节点: `rosrun turtlesim turtlesim_node /turtle1/cmd_vel:=/cmd_vel`

二者可以实现正常通信

使用launch文件

语法：

```
<node pkg="xxx" type="xxx" name="xxx">
    <remap from="原话题" to="新话题" />
</node>
```

```xml
<launch>

    <node pkg="turtlesim" type="turtlesim_node" name="t1" />
    <node pkg="teleop_twist_keyboard" type="teleop_twist_keyboard.py" name="key">
        <remap from="/cmd_vel" to="/turtle1/cmd_vel" />
    </node>

</launch>
```

### 5.3 编码设置话题名称

话题的名称与节点的命名空间、节点的名称是有一定关系的，话题名称大致可以分为三种类型:

- 全局(话题参考ROS系统，与节点命名空间平级)
- 相对(话题参考的是节点的命名空间，与节点名称平级)
- 私有(话题参考节点名称，是节点名称的子级)

结合编码演示具体关系。

#### 1.1全局名称

**格式:**以`/`开头的名称，和节点名称无关

**示例1:**`ros::Publisher pub = nh.advertise<std_msgs::String>("/chatter",1000);`

**结果1:**`/chatter`

#### 1.2相对名称

**格式:**非`/`开头的名称,参考命名空间(与节点名称平级)来确定话题名称

**示例1:**`ros::Publisher pub = nh.advertise<std_msgs::String>("chatter",1000);`

**结果1:**`xxx/chatter`

#### 1.3私有名称

**格式:**以`~`开头的名称

**示例1:**

```
ros::NodeHandle nh("~");
ros::Publisher pub = nh.advertise<std_msgs::String>("chatter",1000);
```

**结果1:**`/xxx/hello/chatter`

## 6 ROS参数名称设置

在ROS中节点名称话题名称可能出现重名的情况，同理参数名称也可能重名。

> 当参数名称重名时，那么就会产生覆盖，如何避免这种情况？

关于参数重名的处理，没有重映射实现，为了尽量的避免参数重名，都是使用为参数名添加前缀的方式，实现类似于话题名称，有全局、相对、和私有三种类型之分。

- 全局(参数名称直接参考ROS系统，与节点命名空间平级)
- 相对(参数名称参考的是节点的命名空间，与节点名称平级)
- 私有(参数名称参考节点名称，是节点名称的子级)

rosrun 在启动节点时，也可以设置参数:

**语法:** rosrun 包名 节点名称 _参数名:=参数值

通过 launch 文件设置参数的方式前面已经介绍过了，可以在 node 标签外，或 node 标签中通过 param 或 rosparam 来设置参数。在 node 标签外设置的参数是全局性质的，参考的是 / ，在 node 标签中设置的参数是私有性质的，参考的是 /命名空间/节点名称。

以 param 标签为例，设置参数

```xml
<launch>

    <param name="p1" value="100" />
    <node pkg="turtlesim" type="turtlesim_node" name="t1">
        <param name="p2" value="100" />
    </node>

</launch>
```

编码的方式可以更方便的设置:全局、相对与私有参数。

------

1.C++实现

在 C++ 中，可以使用 ros::param 或者 ros::NodeHandle 来设置参数。

1.1ros::param设置参数

设置参数调用API是ros::param::set，该函数中，参数1传入参数名称，参数2是传入参数值，参数1中参数名称设置时，如果以 / 开头，那么就是全局参数，如果以 ~ 开头，那么就是私有参数，既不以 / 也不以 ~ 开头，那么就是相对参数。代码示例:

```cpp
ros::param::set("/set_A",100); //全局,和命名空间以及节点名称无关
ros::param::set("set_B",100); //相对,参考命名空间
ros::param::set("~set_C",100); //私有,参考命名空间与节点名称
Copy
```

运行时，假设设置的 namespace 为 xxx，节点名称为 yyy，使用 rosparam list 查看:

```
/set_A
/xxx/set_B
/xxx/yyy/set_C
Copy
```

1.2ros::NodeHandle设置参数

设置参数时，首先需要创建 NodeHandle 对象，然后调用该对象的 setParam 函数，该函数参数1为参数名，参数2为要设置的参数值，如果参数名以 / 开头，那么就是全局参数，如果参数名不以 / 开头，那么，该参数是相对参数还是私有参数与NodeHandle 对象有关，如果NodeHandle 对象创建时如果是调用的默认的无参构造，那么该参数是相对参数，如果NodeHandle 对象创建时是使用:

ros::NodeHandle nh("~")，那么该参数就是私有参数。代码示例:

```cpp
ros::NodeHandle nh;
nh.setParam("/nh_A",100); //全局,和命名空间以及节点名称无关

nh.setParam("nh_B",100); //相对,参考命名空间

ros::NodeHandle nh_private("~");
nh_private.setParam("nh_C",100);//私有,参考命名空间与节点名称
Copy
```

运行时，假设设置的 namespace 为 xxx，节点名称为 yyy，使用 rosparam list 查看:

```
/nh_A
/xxx/nh_B
/xxx/yyy/nh_C
```

## 7 ROS分布式通信

ROS是一个分布式计算环境。一个运行中的ROS系统可以包含分布在多台计算机上多个节点。根据系统的配置方式，任何节点可能随时需要与任何其他节点进行通信。

因此，ROS对网络配置有某些要求：

- 所有端口上的所有机器之间必须有完整的双向连接。
- 每台计算机必须通过所有其他计算机都可以解析的名称来公告自己。

#### **实现**

##### 1.准备

先要保证不同计算机处于同一网络中，最好分别设置固定IP，如果为虚拟机，需要将网络适配器改为桥接模式；

##### 2.配置文件修改

分别修改不同计算机的 /etc/hosts 文件，在该文件中加入对方的IP地址和计算机名:

主机端:

```
从机的IP    从机计算机名
Copy
```

从机端:

```
主机的IP    主机计算机名
Copy
```

设置完毕，可以通过 ping 命令测试网络通信是否正常。

> IP地址查看名: ifconfig
>
> 计算机名称查看: hostname

##### 3.配置主机IP

配置主机的 IP 地址

~/.bashrc 追加

```
export ROS_MASTER_URI=http://主机IP:11311
export ROS_HOSTNAME=主机IP
Copy
```

##### 4.配置从机IP

配置从机的 IP 地址，从机可以有多台，每台都做如下设置:

~/.bashrc 追加

```
export ROS_MASTER_URI=http://主机IP:11311
export ROS_HOSTNAME=从机IP
Copy
```

#### **测试**

1.主机启动 roscore(必须)

2.主机启动订阅节点，从机启动发布节点，测试通信是否正常

3.反向测试，主机启动发布节点，从机启动订阅节点，测试通信是否正常