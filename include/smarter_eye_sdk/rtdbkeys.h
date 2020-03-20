#ifndef RTDBKEYS
#define RTDBKEYS

struct RtdbKey
{
    enum Enumation
    {
        Obsolete_CameraWidth,              //quint16
        Obsolete_CameraHeight,             //quint16
        CameraFrameRate ,         //quint16
        OfflineImagePath,         //string
        VideoSource,
        DownSampleType,

        BuildConfig,
        AdasVersion,
        CollisionAlarmGranularity,
        Obsolete_CollisionAlarmWay,
        AlarmVolume,
        LaneDepartureAlarmGranularity,
        Obsolete_LaneDepartureAlarmWay,

        MemoryUsage,
        CpuUsage,
        AdasBuild,

        Speed,
        Obsolete_ShaftSpeed,
        Obsolete_FuelLevel,

        TemperatureCPU,

        TaskRunnerStatus,        //quint16
        TaskRunnerMemoryUsage,

        Obsolete_DistanceTaskStatus,      //quint16
        Obsolete_DistanceTaskAverage,
        Obsolete_DistanceTaskFrameRate,

        Obsolete_CalibrationTaskAverage,
        Obsolete_CalibrationTaskFrameRate,

        ObstacleExtractionTaskAverage,
        ObstacleExtractionTaskFrameRate,

        LaneTaskStatus,            //quint16
        LaneTaskAverage,
        LaneTaskFrameRate,

        CameraType,
        InstallType,

        Obsolete_SmartServerStatus,        //quint16
        Obsolete_SmartServerMemoryUsage,

        DaemonStatus,             //quint16
        SysStartupMode,
        SmudgeStatus,

        ObstacleAlgType,
        InvalidObstacleFilter,
        ImuAccRange,
        ImuGyrRange,
        ImuSensorOrd,
        ImuPosture,
        ImuDataEnable,
        Model,
        Obsolete_ProtocolVersion,
        Obsolete_FrontVehicleAlarmGranularity,   //C1
        Obsolete_TaskState,  // C1
        Obsolete_WarningAlarmType, // C1
        Obsolete_FloatingMode,
        PlatformVersion,
        CameraFirmwareVersion,
        InitialSetupStatus,
        LaneLearningMode,
        MaxLaneTaskProcessRate,

        AlarmDevice,
        AlarmAdjustEnable,
        SystemVolumeLevel,
        Obsolete_ObsTaskWorkingThreshold,
        FcwWorkingThreshold,
        LdwWorkingThreshold,
        LdwAdjustEnable,
        HmwWorkingThreshold,
        HmwAdjustEnable,
        HmwAlarmThreshold,
        ProductCode,
        MaxSendFrameInterval,
        SerialNum,
        DriveAreaPoints,
        SttpPollInterval,
        UserName,
        OffsetDistance,
        SafetyDistance,
        ObsVersion,
        LaneVersion,
        SdkVersion,
        ThirdDeviceInterface,
        SpeedInterface,
        TurnSignalInterface,
        WiperInterface,
        BrakeInterface,
        ScreenModel,
        ScreenSN,
        ScreenVersion,
        Distance2Ground,
        Distance2LeftGlass,
        Distance2RightGlass,
        Distance2Bumper,
        WheelGap,
        CarHeadHeight,
        VehicleBrand,
        VehicleModel,
        ManufactureDate,
        LicenseNumber,
        HardwareVersion,
        SystemVersion,
        CorrectedDeviceInstallHeight,
        ObsSamplePeriod,
        WinOfflinePath,

        StereoParametersPart1,
        StereoParametersPart2,
        LeftMonoParametersPart1,
        LeftMonoParametersPart2,
        RightMonoParametersPart1,
        RightMonoParametersPart2,

        CanProtocol,
        //Camera Info
        ImageWidth = 1001,
        ImageHeight,
        LensFocus,
        EnvIlluminace,
        LaneLearningProgress,

        HeightLimitTaskAverage,
        HeightLimitTaskFrameRate,
        HeightVersion,

        Real3DToImageRotationMatrixPart1,
        ImageToReal3DRotationMatrixPart1,

        DeviceUpdateProgress,
        Real3DToImageRotationMatrixPart2,
        Real3DToImageRotationMatrixPart3,
        ImageToReal3DRotationMatrixPart2,

        SensorPixelSize,
        BinocularBaseline,
        CameraCalibStatus,
        UpperCPUTemperature,
        LengthDetectionUpperValue,
        PrimaryWarningDistance,
        AdvanceWarningDistance,
        ObsWarningVersion,
        LensSmudgeVersion,
        ScreenLogoID
    };
};

#endif // RTDBKEYS

