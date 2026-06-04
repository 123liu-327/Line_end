#include <flow_end/process_image.h>

#include <cmath>

void process_image()
{
    // ԭͼ     ұ
    int local_thres = 0;
    int block_size = 7;
    int half = block_size / 2;
    int _d = 6;
    int x1 = img_raw.width / 2 - begin_x - after_bizhang_x, y1 = begin_y + after_bizhang_y;
    ipts0_num = sizeof(ipts0) / sizeof(ipts0[0]);
    int find_nums = 5;
    if (after_bizhang_x != 0)
    {
        find_nums = 10;
    }
    for (int _i = 0; _i < find_nums; _i++)
    {
        for (x1 = img_raw.width / 2 - begin_x - after_bizhang_x; x1 > 0; x1--)
        {
            for (int dy = -half; dy <= half; dy++)
            {
                local_thres += (AT_CLIP(&img_raw, x1 + _d, y1 + dy) - AT_CLIP(&img_raw, x1 - _d, y1 + dy));
            }
            local_thres /= block_size;
            if (local_thres >= thres)
                break;
            local_thres = 0;
        }
        if (local_thres >= thres)
            break;
        y1 = begin_y + after_bizhang_y - _i * 25;
    }

    if (local_thres >= thres)
        findline_lefthand_adaptive(&img_raw, block_size, clip_value, x1 - (_d), y1, ipts0, &ipts0_num);
    else
        ipts0_num = 0;
    int x2 = img_raw.width / 2 + begin_x + after_bizhang_x, y2 = begin_y + after_bizhang_y;
    ipts1_num = sizeof(ipts1) / sizeof(ipts1[0]);
    local_thres = 0;
    for (int _i = 0; _i < find_nums; _i++)
    {
        // std::cout << "_i:" << _i << " local_thres:" << local_thres << " y2:" << y2 << std::endl;
        //ROS_INFO("_i: %d local_thres: %d \n", _i, y2);
        for (x2 = img_raw.width / 2 + begin_x + after_bizhang_x; x2 < img_raw.width - 1; x2++)
        {
            for (int dy = -half; dy <= half; dy++)
            {
                local_thres -= (AT_CLIP(&img_raw, x2 + _d, y2 + dy) - AT_CLIP(&img_raw, x2 - _d, y2 + dy));
            }
            local_thres /= block_size;
            // std::cout<<"local_thres:"<<local_thres<<" x2:"<<x2<<std::endl;
            if (local_thres >= thres)
                break;
            local_thres = 0;
        }
        if (local_thres >= thres)
            break;
        y2 = begin_y + after_bizhang_y - _i * 25;
    }
    if (local_thres >= thres)
        findline_righthand_adaptive(&img_raw, block_size, clip_value, x2 + (_d), y2, ipts1, &ipts1_num);
    else
        ipts1_num = 0;

    for (int i = 0; i < ipts0_num; i++)
    {
        rpts0[i][0] = point_map[ipts0[i][1]][ipts0[i][0]][0];
        rpts0[i][1] = point_map[ipts0[i][1]][ipts0[i][0]][1];
    }
    rpts0_num = ipts0_num;

    ROS_INFO("rpts0_num: %d\n", rpts0_num);
    for (int i = 0; i < ipts1_num; i++)
    {
        rpts1[i][0] = point_map[ipts1[i][1]][ipts1[i][0]][0];
        rpts1[i][1] = point_map[ipts1[i][1]][ipts1[i][0]][1];
    }
    rpts1_num = ipts1_num;

    ROS_INFO("rpts1_num: %d\n", rpts1_num);
    //平滑处理
    blur_points(rpts0, rpts0_num, rpts0b, (int)round(line_blur_kernel));
    rpts0b_num = rpts0_num;
    blur_points(rpts1, rpts1_num, rpts1b, (int)round(line_blur_kernel));
    rpts1b_num = rpts1_num;
    //等距采样
    rpts0s_num = sizeof(rpts0s) / sizeof(rpts0s[0]);
    resample_points(rpts0b, rpts0b_num, rpts0s, &rpts0s_num, sample_dist * pixel_per_meter);
    rpts1s_num = sizeof(rpts1s) / sizeof(rpts1s[0]);
    resample_points(rpts1b, rpts1b_num, rpts1s, &rpts1s_num, sample_dist * pixel_per_meter);
    // std::cout << "rpts0s_num:" << rpts0s_num << std::endl;
    ROS_INFO("rpts0s_num: %d\n", rpts0s_num);
    // std::cout << "rpts1s_num:" << rpts1s_num << std::endl;
    ROS_INFO("rpts1s_num: %d\n", rpts1s_num);
    //局部角度估计
    local_angle_points(rpts0s, rpts0s_num, rpts0a, (int)round(angle_dist / sample_dist));
    rpts0a_num = rpts0s_num;
    local_angle_points(rpts1s, rpts1s_num, rpts1a, (int)round(angle_dist / sample_dist));
    rpts1a_num = rpts1s_num;
    //角度变化非极大值抑制
    nms_angle(rpts0a, rpts0a_num, rpts0an, (int)round(0.2 / sample_dist) * 2 + 1);
    rpts0an_num = rpts0a_num;
    nms_angle(rpts1a, rpts1a_num, rpts1an, (int)round(0.2 / sample_dist) * 2 + 1);
    rpts1an_num = rpts1a_num;

    //巡线
    track_leftline(rpts0s, rpts0s_num, rptsc0, (int)round(0.2 / sample_dist), pixel_per_meter * ROAD_WIDTH / 2+Dis_Bias_Left);
    rptsc0_num = rpts0s_num;
    track_rightline(rpts1s, rpts1s_num, rptsc1, (int)round(0.2 / sample_dist), pixel_per_meter * ROAD_WIDTH / 2+Dis_Bias_Right);
    rptsc1_num = rpts1s_num;
    // std::cout << "rptsc0_num:" << rptsc0_num << std::endl;
    ROS_INFO("rptsc0_num: %d\n", rpts1_num);
    // std::cout << "rptsc1_num:" << rptsc1_num << std::endl;
    ROS_INFO("rpts1_num: %d\n", rpts1_num);

    rptsc0e_num = sizeof(rptsc0e) / sizeof(rptsc0e[0]);
    resample_points(rptsc0, rptsc0_num, rptsc0e, &rptsc0e_num, sample_dist * pixel_per_meter);
    rptsc1e_num = sizeof(rptsc1e) / sizeof(rptsc1e[0]);
    resample_points(rptsc1, rptsc1_num, rptsc1e, &rptsc1e_num, sample_dist * pixel_per_meter);
}

ForwardCrossbarResult forward_crossbar_result = {
    false,
    0,
    0,
    0,
    0.0f,
    0.0f,
    0.0f,
    0.0f
};

bool detect_forward_crossbar()
{
    // 每次调用都先清空结果，避免上一帧检测结果误用到当前帧。
    forward_crossbar_result = {
        false,
        0,
        0,
        0,
        0.0f,
        0.0f,
        0.0f,
        0.0f
    };

    if (img_raw.data == nullptr || pixel_per_meter <= 1.0f) {
        return false;
    }

    // 只在图像正前方的中距离区域找横线，减少普通边线和停车区边缘误触发。
    const int image_center_x = RESULT_COL / 2;
    const int roi_half_width = 160;
    const int roi_x_min = clip(image_center_x - roi_half_width, 0, RESULT_COL - 1);
    const int roi_x_max = clip(image_center_x + roi_half_width, 0, RESULT_COL - 1);
    const int roi_y_min = clip(static_cast<int>(RESULT_ROW * 0.35f), 0, RESULT_ROW - 1);
    const int roi_y_max = clip(static_cast<int>(RESULT_ROW * 0.75f), 0, RESULT_ROW - 1);
    const int min_width_px = std::max(8, static_cast<int>(std::round(0.25f * pixel_per_meter)));
    const int center_tolerance_px = std::max(4, static_cast<int>(std::round(0.12f * pixel_per_meter)));

    int best_left = 0;
    int best_right = 0;
    int best_y = 0;
    int best_width = 0;
    int best_center_error = RESULT_COL;

    for (int y = roi_y_min; y <= roi_y_max; ++y) {
        int run_start = -1;
        for (int x = roi_x_min; x <= roi_x_max + 1; ++x) {
            const bool in_roi = x <= roi_x_max;
            // ImageUsed 是当前巡线使用的二值/逆透视图；亮像素视为候选线像素。
            const bool is_line_pixel = in_roi && ImageUsed[y][x] > 128;

            if (is_line_pixel && run_start < 0) {
                run_start = x;
            }

            if ((!is_line_pixel || !in_roi) && run_start >= 0) {
                const int run_end = x - 1;
                const int width = run_end - run_start + 1;
                const int center_x = (run_start + run_end) / 2;
                const int center_error = std::abs(center_x - image_center_x);

                if (width >= min_width_px && center_error <= center_tolerance_px) {
                    // 优先选择更宽的横线；宽度相同时选更靠近车体中心的线段。
                    const bool better_width = width > best_width;
                    const bool same_width_better_center =
                        width == best_width && center_error < best_center_error;
                    if (better_width || same_width_better_center) {
                        best_left = run_start;
                        best_right = run_end;
                        best_y = y;
                        best_width = width;
                        best_center_error = center_error;
                    }
                }

                run_start = -1;
            }
        }
    }

    if (best_width <= 0) {
        return false;
    }

    const int center_x = (best_left + best_right) / 2;
    const int center_y = best_y;
    // point_map 把图像点映射到当前工程使用的逆透视平面坐标。
    const float map_x = static_cast<float>(point_map[center_y][center_x][0]);
    const float map_y = static_cast<float>(point_map[center_y][center_x][1]);
    const float ref_x = RESULT_COL / 2.0f;
    const float ref_y = RESULT_ROW + 10.0f;

    // long_m/lat_m 和停车逻辑的距离换算保持一致，便于后续接入 Y 岔路靠近流程。
    forward_crossbar_result.found = true;
    forward_crossbar_result.center_x = center_x;
    forward_crossbar_result.center_y = center_y;
    forward_crossbar_result.width_px = best_width;
    forward_crossbar_result.map_x = map_x;
    forward_crossbar_result.map_y = map_y;
    forward_crossbar_result.long_m = -(map_y - ref_y) / pixel_per_meter;
    forward_crossbar_result.lat_m = -(map_x - ref_x) / pixel_per_meter;

    return true;
}
