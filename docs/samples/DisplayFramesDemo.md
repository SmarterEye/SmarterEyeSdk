# DisplayFramesDemo

从设备获取双目相机左右校准图和深度图，在窗口显示。
此Demo位于 `<sdk>/samples/DisplayFramesDemo` 下。

!> 该Demo依赖`OpenCV`库，使用OpenCV的`highgui`模块提供方法显示图像。  
在使用此Demo前，需要安装OpenCV库。

!> 经测试，在Ubuntu系统下，基于我们提供的SDK，使用OpenCV的highgui模块的函数，必须在Qt的EventLoop线程下执行，否则程序会异常退出。在Windows环境下，则不存在此问题。  
为此，我们提供了一个接口[StereoCameraDemo::invokeInLoopThread](api/stereo_camera?id=%e7%89%b9%e6%ae%8a%e5%87%bd%e6%95%b0)，此函数接收一个函数对象，并在Qt的EventLoop线程下执行该函数。

## 代码逻辑

* 与设备建立连接
* 请求并接收`左右校准图` 和 `深度图`数据
* 调用OpenCV的highgui模块的函数，在窗口显示图像

## 参考代码片段
```C++
int main(int argc, char *argv[])
{
    StereoCamera *camera = StereoCamera::connect("192.168.1.251");
    std::unique_ptr<FrameMonitor> frameMonitor(new FrameMonitor);

    camera->enableTasks(TaskId::NoTask);
    camera->requestFrame(frameMonitor.get(), FrameId::CalibLeftCamera | FrameId::CalibRightCamera | FrameId::Disparity);

    cv::Mat combinedFrame;
    std::function<int()> draw_func = [&frameMonitor, &combinedFrame]() -> int {
        cv::Mat leftFrame = frameMonitor->getFrameMat(FrameId::CalibLeftCamera);
        cv::Mat rightFrame = frameMonitor->getFrameMat(FrameId::CalibRightCamera);
        cv::Mat disparity = frameMonitor->getFrameMat(FrameId::Disparity);

        if (leftFrame.type() == rightFrame.type()
                && !leftFrame.empty() && !rightFrame.empty()) {
            cv::hconcat(leftFrame, rightFrame, combinedFrame);
            cv::imshow("Frame", combinedFrame);
        }

        if (!disparity.empty()) {
            cv::imshow("Disparity", disparity);
        }

        return cv::waitKey(80);
    };

    camera->invokeInLoopThread([]{
        cv::namedWindow("Frame");
        cv::namedWindow("Disparity");
    });

    // main thread loop for drawing images
    while (true) {
        frameMonitor->waitForFrames();  // wait for frames ready

        int key = 0;
        camera->invokeInLoopThread([&key, &draw_func]{
            key = draw_func();
        });

        if (key == 27) {
            // press Esc to close
            break;
        }
    }

    camera->invokeInLoopThread([]{
        cv::destroyAllWindows();
    });

    return 0;
}
```