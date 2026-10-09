# Báo cáo bài nộp — Day 23 Sensor Fusion Lab

> Điền file này rồi commit. Cách nộp: [hướng dẫn nộp](../SUBMISSION.md).

## Thông tin học viên

- Họ tên: Trần Long Khánh
- MSSV: 2A202602538
- Email: khanhtrankuri@gmail.com
- Link repo (fork): https://github.com/khanhtrankuri/K4-Track4-Day23-TranLongKhanh-2A202602538-SensorFusion
- Commit hash nộp (`git rev-parse HEAD`): b5a2ee0973ff1d253a94155b573cc3dac98d8508 (CP5 code + artifacts; dùng hash CP6 mới nhất khi nộp LMS)

## Tóm tắt kết quả

- `fusion_mode` (bắt buộc `compare`), `frames`, `segment`, `seed`: `compare`, `[0, 198]`, `training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord`, `0`.
- `detection.precision`, `detection.recall`, `detection.tp/fp/fn`: `0.9700934579`, `0.7004048583`, `519/16/222`.
- `tracking.lidar.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: `0.1502780648 m`, `502`, `11.3369153778 m²`, `0`, `239`, `2.5226130653`.
- `tracking.fused.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: `0.1358477507 m`, `502`, `9.2642149137 m²`, `0`, `239`, `2.5226130653`.
- Giải thích khác biệt hai mode, đọc RMSE cùng số ghép và ghost/miss: fused giảm RMSE khoảng `0.01443 m` và giảm tổng sai số bình phương khoảng `2.07270 m²`. Hai mode vẫn có cùng `502` matches, `0` ghost, `239` miss và cùng số track confirmed trung bình, nên cải thiện đến từ camera update tinh chỉnh vị trí các track đã được LiDAR tạo/quản lý, không phải do fused tạo thêm track hay thay đổi coverage.

Chạy từ root repo:

```bash
fusion-run-lab --config student/config/paths.yaml --fusion compare --seed 0
```

`rmse = sqrt(sum_sq_err/matches)` trên vị trí 3D của confirmed tracks ghép
một-một với GT xe trong cửa sổ BEV, gate XY **2.0 m**; `null` nếu không có cặp.
Camera dùng tâm hộp 2D ground-truth FRONT có nhiễu seeded, **không** dùng camera
detector. Kết quả này không đo hiệu quả một perception system độc lập với GT.

`grade_run.log` là JSONL, mỗi `(mode,frame)` đúng một record với các trường:
`mode`, `frame`, `det_tp`, `det_fp`, `det_fn`, `valid_gt`, `confirmed`, `matches`,
`sum_sq_err`, `ghosts`, `misses`. Đảm bảo `matches+ghosts==confirmed` và
`matches+misses==valid_gt`; tổng/trung bình record phải khớp `metrics.json`.
File per-mode `metrics_lidar.json`, `metrics_fused.json`, `grade_run_lidar.log`,
`grade_run_fused.log` được giữ để đối chiếu.

## Giải thích ngắn (Parts E–H — tự viết)

1. LiDAR đo trực tiếp vị trí 3D nên `z` là cột `(x,y,z)` và `R` là ma trận chéo 3×3 theo `sigma_lidar_x/y/z²`; mô hình đo gần tuyến tính. Camera đo tâm ảnh 2D nên `z=(u,v)`, `R=diag(sigma_cam_i²,sigma_cam_j²)` theo pixel, còn `h(x)` là phép chiếu pinhole phi tuyến; EKF dùng Jacobian 2×6 từ calibration tại trạng thái hiện tại (`camera_fusion.py`, `kalman.py`).
2. Mahalanobis chuẩn hóa residual theo độ bất định `S=HPHᵀ+R`, nên cùng một sai lệch Euclidean có ý nghĩa khác nhau khi track/sensor có covariance khác nhau. Cổng χ² loại các cặp không tương thích trước greedy assignment, tránh một outlier chiếm measurement và kéo EKF sang trạng thái sai (`association.py`).
3. Đây là **track-then-fuse**: mỗi frame predict một lần, association/update LiDAR trước rồi mới association/update camera trên cùng danh sách track. `grade_run.log` có 199 record `lidar` rồi 199 record `fused`; detection counts giống nhau ở hai mode, còn mỗi run duy trì tracker riêng để so sánh công bằng.
4. Sai extrinsic/intrinsic làm `h(x)` bị lệch có hệ thống, khiến innovation camera lớn và có hướng thiên lệch qua nhiều frame. Mahalanobis tăng; nếu vượt cổng χ² thì measurement bị loại, còn nếu vẫn lọt cổng thì camera update kéo state sai và có thể làm RMSE fused tăng.
5. Khi `meas_list` rỗng không thể suy ra modality từ measurement, nhưng manager vẫn phải biết đây là lượt LiDAR hay camera. Lượt LiDAR rỗng phải trừ score cho track trong FOV, xét xóa và vẫn hoàn tất lifecycle; camera rỗng phải giữ nguyên lifecycle. Vì vậy `sensor` được truyền tường minh và `manage_tracks(...)` luôn được gọi. Chỉ LiDAR có quan sát 3D đủ để birth/score/delete; camera chỉ refine EKF của track đã tồn tại (`association.py`, `manager.py`).
6. Track bắt đầu ở `initialized`, score `1/window`; mỗi LiDAR hit cộng `1/window` (cap 1), và xác nhận khi `score > confirmed_threshold`. Hit trước ngưỡng chuyển sang `tentative`; track đã `confirmed` không bị hạ trạng thái sau miss. LiDAR miss trong FOV trừ `1/window`. Xóa nếu `Pxx` hoặc `Pyy > max_P`; hoặc confirmed có `score < delete_threshold`; hoặc track chưa confirmed có `score <= 0` (`track_management.py`).

## Bonus (không bắt buộc)

Liệt kê phần bonus đã làm, file bằng chứng trong `student/bonus/` và kết quả chính
(xem [RUBRIC.md](../RUBRIC.md) mục 2). Không làm thì ghi "Không".

- Không.

## Khai báo sử dụng AI (bắt buộc)

Ghi rõ, kể cả khi không dùng ("Không dùng AI"). Xem [RULES.md](../RULES.md) mục 2.

- Công cụ đã dùng (ChatGPT, Copilot, Claude, …): OpenAI Codex (ChatGPT).
- Dùng cho phần nào (hàm, câu hỏi, debug): đọc yêu cầu; hỗ trợ triển khai và rà soát các hàm Part E–H; chẩn đoán môi trường Windows/env `vin`; hỗ trợ diễn giải kết quả trong báo cáo.
- Cách bạn đã kiểm tra lại (pytest, chạy Waymo, đối chiếu công thức): chạy `128 passed` bằng env `vin` với Python UTF-8; chạy thật `--fusion compare --seed 0` đủ frame 0–198; đối chiếu `F`, `Q`, innovation, `S`, Kalman gain, projection pinhole, Mahalanobis/χ² và các invariant metrics/log.

## Checklist nộp

- [x] **Part E–H** trong `workspace/` đã implement; `pytest student/tests -q` không còn `failed`/`xfailed`
- [x] Part A–D: không sửa
- [x] Lần chạy chấm điểm: `--fusion compare --seed 0`, `frame_start: 0`, `frame_end: 198`
- [x] Đã commit `student/artifacts/metrics*.json` và `student/artifacts/grade_run*.log` (không sửa tay)
- [x] Đã điền đủ file này, gồm khai báo AI
- [x] Không commit dữ liệu Waymo, weights, `paths.yaml`, API key
- [ ] `python tools/check_submission.py` báo `KẾT QUẢ: SẴN SÀNG NỘP`
- [ ] Đã push và nộp link repo + commit hash trên LMS ([hướng dẫn nộp](../SUBMISSION.md))
