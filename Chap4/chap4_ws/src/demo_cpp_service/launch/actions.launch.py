import launch
import launch.launch_description_sources
import launch_ros
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    action_declare_arg_startup_rqt = launch.actions.DeclareLaunchArgument('startup_rqt',default_value="False")#启动参数声明
    startup_rqt = launch.substitutions.LaunchConfiguration('startup_rqt',default="False")#赋值启动参数


    #动作1：启动其他launch
    multisim_launch_path = [get_package_share_directory('turtlesim'),'/launch','/multisim.launch.py']
    action_include_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            multisim_launch_path
        )
    )

    #动作2：打印数据
    action_log_info = launch.actions.LogInfo(msg=str(multisim_launch_path))

    #动作3：执行进程（命令行，例如ros2 topic list）
    action_topic_list = launch.actions.ExecuteProcess(
        #cmd=['ros2','topic','list']
        condition = launch.conditions.IfCondition(startup_rqt), #启动条件
        cmd=['rqt']
    )

    #动作4：组织动作成组，把多个动作放到一组
    action_group = launch.actions.GroupAction([
        #动作5：定时器
        launch.actions.TimerAction(period=2.0,actions=[action_include_launch]),
        launch.actions.TimerAction(period=4.0,actions=[action_topic_list])
    ])

    return launch.LaunchDescription([
        #action动作
        action_log_info,
        action_group,
       
    ])
