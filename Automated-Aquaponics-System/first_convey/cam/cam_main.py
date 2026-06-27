# cam_main.py
from tray import TrayConfig, TrayAngleStreamer

def main():
    cfg = TrayConfig(
        cam_index=1,
        use_dshow=True,

        # OpenCR 포트 고정
        force_port="COM20",
        baud=115200,

        # 캘리브레이션 파일
        calib_path="camera_calib.npz",  # 같은 폴더에 두면 됨
        alpha=1.0,
        crop=True,
        use_remap=True,

        # 전송/검출 파라미터
        min_area=4000,
        canny1=150,
        canny2=350,
        send_interval=0.05,
        send_delta_deg=0.3,

        show_windows=True,
        print_angle_to_console=False,
    )

    app = TrayAngleStreamer(cfg)
    app.run()

if __name__ == "__main__":
    main()
