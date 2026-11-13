import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

QAIRT250_DEFAULT_ROOT = "/opt/qairt-2.50"
SYSTEM_QNN_HTP = "/usr/lib/libQnnHtp.so"


def _resolve_qairt(context):
    """Resolve the qairt launch arg to (backend .so path, extra env or None, arg)."""
    arg = LaunchConfiguration("qairt").perform(context)
    if arg == "system":
        return SYSTEM_QNN_HTP, None, arg
    if arg == "auto":
        candidate = os.path.join(QAIRT250_DEFAULT_ROOT, "lib", "libQnnHtp.so")
        root = QAIRT250_DEFAULT_ROOT if os.path.exists(candidate) else None
    else:
        root = arg
    if root is None:
        return SYSTEM_QNN_HTP, None, arg
    backend = os.path.join(root, "lib", "libQnnHtp.so")
    if not os.path.exists(backend):
        raise RuntimeError(
            f"yolo_npu: qairt:={arg} -> {backend} does not exist "
            f"(expected <root>/lib/libQnnHtp.so)"
        )
    ld_path = [os.path.join(root, "lib")]
    if os.environ.get("LD_LIBRARY_PATH"):
        ld_path.append(os.environ["LD_LIBRARY_PATH"])
    env = {
        "LD_LIBRARY_PATH": ":".join(ld_path),
        "ADSP_LIBRARY_PATH": os.path.join(root, "dsp")
        + ";/usr/lib/dsp/cdsp;/usr/lib/rfsa/adsp;/dsp",
    }
    return backend, env, arg


def _nodes(context):
    backend, env, qairt_arg = _resolve_qairt(context)

    input_image = LaunchConfiguration("input_image")
    enable_camera = LaunchConfiguration("enable_camera")
    model = LaunchConfiguration("model")
    enable_overlay = LaunchConfiguration("enable_overlay")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # IMX577 colour camera -> cam0_stream1 (NV12) fed straight into the
    # preprocess node. The OV9282 mono camera cannot feed this pipeline:
    # qrb_ros_transport rejects mono8, and its NV12 luma plane is greyscale
    # anyway, so detection quality would be poor.
    camera = ComposableNode(
        package="qrb_ros_camera",
        plugin="qrb_ros::camera::CameraNode",
        name="camera_node",
        parameters=[
            {
                "camera_info_path": PathJoinSubstitution(
                    [
                        FindPackageShare("qrb_ros_camera"),
                        "config",
                        "camera_info_imx577.yaml",
                    ]
                ),
                "camera_id": 0,
                "stream_size": 1,
                "stream_name": ["stream1"],
                "stream1": {"height": 1080, "width": 1920, "fps": 30},
                "use_sim_time": use_sim_time,
            }
        ],
        remappings=[("cam0_stream1", input_image)],
        condition=IfCondition(enable_camera),
    )

    preprocess = ComposableNode(
        package="qrb_ros_cv_tensor_common_process",
        plugin="qrb_ros::cv_tensor_common_process::CvTensorCommonProcessNode",
        name="yolo_preprocess_node",
        parameters=[
            {"target_res": "640x640"},
            # AI Hub export is compiled with --quantize_io: the graph input is
            # uint8 raw pixels (scale=1/255), NOT normalized float32.
            {"normalize": False},
            {"tensor_fmt": "nhwc"},
            {"data_type": "uint8"},
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("input_image", input_image),
            ("encoded_image", "qrb_inference_input_tensor"),
        ],
    )

    inference = ComposableNode(
        package="qrb_ros_nn_inference",
        plugin="qrb_ros::nn_inference::QrbRosInferenceNode",
        name="nn_inference_node",
        parameters=[
            {"model_path": model},
            {"backend_option": backend},
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("qrb_inference_output_tensor", "yolo_segment_tensor_output"),
        ],
    )

    overlay = ComposableNode(
        package="qrb_ros_yolo_process",
        plugin="qrb_ros::yolo_process::YoloSegOverlayNode",
        name="yoloe_overlay_node",
        parameters=[
            {
                "target_res": "640x640",
                "mask_res": "160x160",
                "use_sim_time": use_sim_time,
            }
        ],
        remappings=[
            ("yolo_segment_result", "yoloe/detections"),
            ("yolo_segment_overlay", "yoloe/overlay"),
        ],
        condition=IfCondition(enable_overlay),
    )

    container_env = env if env else {}
    return [
        LogInfo(msg=f"yolo_npu: QAIRT backend {backend} (qairt:={qairt_arg})"),
        ComposableNodeContainer(
            name="yoloe_npu_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[camera, preprocess, inference, overlay],
            additional_env=container_env,
            # The container holds camera + inference: if it dies, inference
            # is over, so shut the whole launch down and let the unit fail
            # instead of leaving the postprocess alone in an "active" unit.
            on_exit=Shutdown(reason="yoloe container exited"),
            output="screen",
        ),
    ]


def generate_launch_description():
    """YOLOE open-vocabulary segmentation on the Hexagon NPU.

    Pipeline (topics, all on-domain):

      cam0_stream1  (sensor_msgs/Image, NV12 from the IMX577 colour camera)
        -> CvTensorCommonProcessNode   : resize/normalize -> qrb_inference_input_tensor
        -> QrbRosInferenceNode         : QNN HTP (.bin)  -> yolo_segment_tensor_output
        -> yoloe_seg_postprocess.py    : NMS + mask decode -> yoloe/detections
        -> YoloSegOverlayNode (optional) -> yoloe/overlay

    The .bin must be an AI Hub `yoloe_seg` qnn_context_binary compiled for the
    RB3 Gen 2 (QCS6490, Hexagon V68). Bundled /opt/model/*.bin files are
    compiled for V73 and do NOT run on this unit.

    QAIRT runtime: AI Hub exports are now built with QAIRT 2.50, which the
    packaged 2.49 runtime refuses ("Using newer context binary on old SDK"),
    so the fleet also carries a side-by-side 2.50 stack under
    /opt/qairt-2.50 (installed by qb3rt_base, not a system package). The
    `qairt` launch arg selects the runtime:

      auto    (default) use /opt/qairt-2.50 if it has lib/libQnnHtp.so,
              else the system runtime in /usr/lib
      system  force /usr/lib/libQnnHtp.so with no env changes
      <dir>   explicit QAIRT root containing lib/ and dsp/

    A non-system root also sets LD_LIBRARY_PATH=<root>/lib and
    ADSP_LIBRARY_PATH=<root>/dsp;... on the container so the DSP skels come
    from the same stack. The resolved backend is logged at startup.

      ros2 launch QB3rt yolo_npu.launch.py \\
          model:=/home/ubuntu/models/yoloe_coco80_qa249.bin \\
          label_file:=.../config/yoloe_coco80.yaml

    For the lab-vocab model use yoloe_lab_qa249.bin + yoloe_lab.yaml. The vocabulary
    is baked into the .bin at export time (--prompt-text); label_file must list
    the same classes in the same order.
    """
    label_file = LaunchConfiguration("label_file")
    quant_params_file = LaunchConfiguration("quant_params_file")
    score_thres = LaunchConfiguration("score_thres")
    iou_thres = LaunchConfiguration("iou_thres")
    use_sim_time = LaunchConfiguration("use_sim_time")

    postprocess = ExecuteProcess(
        cmd=[
            "python3",
            PathJoinSubstitution(
                [FindPackageShare("QB3rt"), "scripts", "yoloe_seg_postprocess.py"]
            ),
            "--ros-args",
            "-p", "input_topic:=yolo_segment_tensor_output",
            "-p", "output_topic:=yoloe/detections",
            "-p", ["label_file:=", label_file],
            "-p", ["quant_params_file:=", quant_params_file],
            "-p", ["score_thres:=", score_thres],
            "-p", ["iou_thres:=", iou_thres],
            "-p", ["use_sim_time:=", use_sim_time],
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_camera",
                default_value="true",
                description="Start the IMX577 colour camera in this launch. "
                "Set false when cam0_stream1 is already published elsewhere.",
            ),
            DeclareLaunchArgument(
                "input_image",
                default_value="cam0_stream1",
                description="Image topic fed to the preprocess node "
                "(NV12 from the IMX577; the camera remap defaults to the same).",
            ),
            DeclareLaunchArgument(
                "model",
                default_value="/home/ubuntu/models/yoloe_coco80_qa249.bin",
                description="AI Hub yoloe_seg qnn_context_binary (.bin) path",
            ),
            DeclareLaunchArgument(
                "label_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("QB3rt"), "config", "yoloe_coco80.yaml"]
                ),
                description="ultralytics-style names yaml matching the baked vocab",
            ),
            DeclareLaunchArgument(
                "quant_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("QB3rt"), "config", "yoloe_coco80_quant.yaml"]
                ),
                description="Per-tensor scale/offset for the quantized .bin "
                "(from qnn-context-binary-utility --json_file); must match model.",
            ),
            DeclareLaunchArgument("score_thres", default_value="0.25"),
            DeclareLaunchArgument("iou_thres", default_value="0.45"),
            DeclareLaunchArgument("enable_overlay", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "qairt",
                default_value="auto",
                description="QAIRT runtime: 'auto' uses /opt/qairt-2.50 when "
                "present else the system runtime; 'system' forces "
                "/usr/lib/libQnnHtp.so; any other value is a QAIRT root dir "
                "with lib/ and dsp/ subdirs.",
            ),
            OpaqueFunction(function=_nodes),
            postprocess,
        ]
    )
