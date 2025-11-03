from datetime import datetime, timedelta

import matplotlib.pyplot as plt
import pandas as pd

def convert_traffic_level(congestion):
    """Convert traffic level to numeric value (0.0-1.0)
    
    Args:
        congestion: String level ("low"/"medium"/"high"/"jam", case-insensitive) 
                   or numeric value (0.0-1.0)
    
    Returns:
        float: Numeric congestion level (0.0-1.0) or None if invalid
    """
    if isinstance(congestion, str):
        level_map = {
            "low": 0.2,
            "medium": 0.5, 
            "high": 0.7,
            "jam": 0.9
        }
        return level_map.get(congestion.lower(), None)
    elif isinstance(congestion, (int, float)) and 0.0 <= congestion <= 1.0:
        return float(congestion)
    return None

def analyze_task_processing_times(
    task_data_list, title="Task Processing Time Analysis", traffic_data_list=None
):
    """Analyze task processing times and create visualization from raw data.

    Args:
        task_data_list: List of dicts with keys:
                        - 'start_time': datetime or float (seconds from base time)
                        - 'processing_time': float (seconds)
                        OR list of tuples (start_time, processing_time)
        title: Title for the analysis plot
        traffic_data_list: Optional. List of dictionaries with keys:
                          - 'start_time': datetime or float (seconds from base time)
                          - 'congestion_level': string ("low"/"medium"/"high"/"jam", case-insensitive) 
                                             OR float (0.0-1.0)

    Returns:
        pandas.DataFrame: 10-min binned stats (seconds-based μ/σ) filtered by n>0
    """

    print("=== Task Processing Time Analysis ===")
    print(f"Input data points: {len(task_data_list)}")

    if not task_data_list:
        print("No data provided.")
        return None

    # ---------------------------
    # Normalize input data
    # ---------------------------
    processed_data = []
    base_time = datetime.now().replace(hour=0, minute=0, second=0, microsecond=0)

    for item in task_data_list:
        if isinstance(item, dict):
            start_time = item.get("start_time")
            processing_time = item.get("processing_time")
        elif isinstance(item, (tuple, list)) and len(item) >= 2:
            start_time, processing_time = item[0], item[1]
        else:
            continue

        if isinstance(start_time, (int, float)):
            start_datetime = base_time + timedelta(seconds=start_time)
        elif isinstance(start_time, datetime):
            start_datetime = start_time
        else:
            continue

        if processing_time is not None:
            processed_data.append(
                {"start_time": start_datetime, "processing_time": processing_time}
            )

    print(f"Valid data points for analysis: {len(processed_data)}")
    if not processed_data:
        print("No valid data found.")
        return None

    # ---------------------------
    # DataFrame & resample(30min)
    # ---------------------------
    df = pd.DataFrame(processed_data).set_index("start_time").sort_index()

    stats = df.resample("30min").agg(
        mu=("processing_time", "mean"),
        sigma=("processing_time", "std"),   # ddof=1 (표본 표준편차)
        n=("processing_time", "count"),
    )

    # μ ± σ / μ ± 2σ bands (초 단위)
    stats["lo_1sigma"] = stats["mu"] - stats["sigma"]
    stats["hi_1sigma"] = stats["mu"] + stats["sigma"]
    stats["lo_2sigma"] = stats["mu"] - 2 * stats["sigma"]
    stats["hi_2sigma"] = stats["mu"] + 2 * stats["sigma"]

    # 유효 구간만 (n>0)
    stats_filtered = stats.loc[stats["n"] > 0].copy()
    if len(stats_filtered) == 0:
        print("No data after filtering.")
        return None

    # ---------------------------
    # Plot: Processing time boxplot by time periods
    # ---------------------------
    SEC2MIN = 1.0 / 60.0

    # 원본 데이터를 30분 구간별로 그룹화하여 boxplot 생성
    df_plot = df.copy()
    df_plot["processing_time_min"] = df_plot["processing_time"] * SEC2MIN
    df_plot["time_period"] = df_plot.index.floor("30min")

    # 각 시간 구간별 처리 시간 데이터 수집
    time_periods = sorted(df_plot["time_period"].unique())
    processing_time_data = []
    period_labels = []

    for period in time_periods:
        period_times = df_plot[df_plot["time_period"] == period]["processing_time_min"]
        if len(period_times) > 0:  # 데이터가 있는 구간만
            processing_time_data.append(period_times.values)
            period_labels.append(period.strftime("%H:%M"))

    if not processing_time_data:
        print("No data available for plotting.")
        return stats_filtered

    # ---------------------------
    # Process traffic data if provided
    # ---------------------------
    traffic_df = None
    if traffic_data_list:
        traffic_processed_data = []
        for item in traffic_data_list:
            if isinstance(item, dict):
                start_time = item.get("start_time")
                congestion = item.get("congestion_level")
            elif isinstance(item, (tuple, list)) and len(item) >= 2:
                start_time, congestion = item[0], item[1]
            else:
                continue

            if isinstance(start_time, (int, float)):
                start_datetime = base_time + timedelta(seconds=start_time)
            elif isinstance(start_time, datetime):
                start_datetime = start_time
            else:
                continue

            congestion_value = convert_traffic_level(congestion)
            if congestion_value is not None:
                traffic_processed_data.append({"start_time": start_datetime, "congestion": congestion_value})

        if traffic_processed_data:
            traffic_df = pd.DataFrame(traffic_processed_data).set_index("start_time").sort_index()

    # Create subplot layout: main plot + optional traffic overlay
    if traffic_df is not None:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), height_ratios=[3, 1], sharex=True)
        main_ax = ax1
    else:
        fig, main_ax = plt.subplots(1, 1, figsize=(12, 6))
        ax2 = None

    # Boxplot 생성
    bp = main_ax.boxplot(processing_time_data, labels=period_labels, patch_artist=True,
                    showmeans=True, meanline=True,
                    boxprops=dict(facecolor='lightgreen', alpha=0.7),
                    meanprops=dict(color='red', linewidth=2),
                    medianprops=dict(color='darkgreen', linewidth=2),
                    whiskerprops=dict(color='black'),
                    capprops=dict(color='black'),
                    flierprops=dict(marker='o', markerfacecolor='red', markersize=3, alpha=0.5))

    # 제목/레이블
    main_ax.set_title(f"{title} - Processing Time Distribution (30-min intervals)", fontsize=14)
    if traffic_df is None:
        main_ax.set_xlabel("Time Period", fontsize=12)
    main_ax.set_ylabel("Processing Time (minutes)", fontsize=12)

    # 격자 및 축 포맷
    main_ax.grid(True, alpha=0.3, axis='y')

    # x축 레이블 회전 (너무 많으면)
    if len(period_labels) > 8 and traffic_df is None:
        plt.xticks(rotation=45)

    # 범례 추가 (평균선과 중앙값선 설명)
    main_ax.plot([], [], color='red', linewidth=2, label='Mean')
    main_ax.plot([], [], color='darkgreen', linewidth=2, label='Median')
    main_ax.legend(loc='upper right')

    # ---------------------------
    # Add traffic congestion stacked area chart if data is available
    # ---------------------------
    if traffic_df is not None and ax2 is not None:
        # Resample traffic data to match time periods
        traffic_resampled = traffic_df.resample("30min").mean()
        
        # Create x positions for chart
        x_positions = list(range(len(period_labels)))
        traffic_values = []
        
        # Align traffic data with processing time periods
        for period_str in period_labels:
            period_dt = datetime.strptime(period_str, "%H:%M").time()
            # Find matching traffic data for this period
            matching_traffic = None
            for traffic_time in traffic_resampled.index:
                if traffic_time.time().replace(second=0, microsecond=0) == period_dt:
                    matching_traffic = traffic_resampled.loc[traffic_time, 'congestion']
                    break
            
            if matching_traffic is not None:
                traffic_values.append(matching_traffic)
            else:
                traffic_values.append(0.2)  # Default to low traffic if no data
        
        # Define color mapping for traffic levels
        colors = {
            'low': '#1f77b4',      # Blue
            'medium': '#ff7f0e',   # Orange  
            'high': '#2ca02c',     # Green
            'jam': '#d62728'       # Red
        }
        
        # Create stacked area chart based on traffic levels
        for i, x_pos in enumerate(x_positions):
            congestion_level = traffic_values[i]
            
            # Determine traffic level and color
            if congestion_level <= 0.35:
                level = 'low'
                color = colors['low']
            elif congestion_level <= 0.6:
                level = 'medium' 
                color = colors['medium']
            elif congestion_level <= 0.8:
                level = 'high'
                color = colors['high']
            else:
                level = 'jam'
                color = colors['jam']
            
            # Draw bar for this time period (100% height, colored by level)
            ax2.bar(x_pos, 1.0, width=0.8, color=color, alpha=0.7, 
                   edgecolor='white', linewidth=0.5)
        
        ax2.set_ylabel("Traffic\nLevel", fontsize=10)
        ax2.set_xlabel("Time Period", fontsize=12)
        ax2.set_ylim(0, 1)
        ax2.set_xticks(x_positions)
        ax2.set_xticklabels(period_labels)
        ax2.grid(True, alpha=0.3, axis='x')
        
        # x축 레이블 회전
        if len(period_labels) > 8:
            plt.setp(ax2.xaxis.get_majorticklabels(), rotation=45)
        
        # Create legend with level colors
        from matplotlib.patches import Patch
        legend_elements = [
            Patch(facecolor=colors['low'], label='Low'),
            Patch(facecolor=colors['medium'], label='Medium'),
            Patch(facecolor=colors['high'], label='High'),
            Patch(facecolor=colors['jam'], label='Jam')
        ]
        ax2.legend(handles=legend_elements, loc='upper right', fontsize=8)
        
        # Remove y-axis ticks since it's categorical
        ax2.set_yticks([])

    plt.tight_layout()
    return stats_filtered


def analyze_task_processing_speed(
    task_data_list, title="Task Processing Speed Analysis", traffic_data_list=None
):
    """Analyze task processing speed and create visualization from raw data.

    Args:
        task_data_list: List of dictionaries with keys:
                       - 'start_time': datetime or float (seconds from base time)
                       - 'speed': float
                       OR list of tuples (start_time, speed)
        title: Title for the analysis plot
        traffic_data_list: Optional. List of dictionaries with keys:
                          - 'start_time': datetime or float (seconds from base time)
                          - 'congestion_level': string ("low"/"medium"/"high"/"jam", case-insensitive)
                                             OR float (0.0-1.0)
                          OR list of tuples (start_time, congestion_level)

    Returns:
        pandas.DataFrame: Filtered statistics for further analysis
    """
    print("=== Task Processing Speed Analysis ===")
    print(f"Input data points: {len(task_data_list)}")

    if not task_data_list:
        print("No data provided.")
        return None

    # ---------------------------
    # Normalize input data
    # ---------------------------
    processed_data = []
    base_time = datetime.now().replace(hour=0, minute=0, second=0, microsecond=0)

    for item in task_data_list:
        if isinstance(item, dict):
            start_time = item.get("start_time")
            speed = item.get("speed")
        elif isinstance(item, (tuple, list)) and len(item) >= 2:
            start_time, speed = item[0], item[1]
        else:
            continue

        if isinstance(start_time, (int, float)):
            start_datetime = base_time + timedelta(seconds=start_time)
        elif isinstance(start_time, datetime):
            start_datetime = start_time
        else:
            continue

        if speed is not None:
            processed_data.append({"start_time": start_datetime, "speed": speed})

    print(f"Valid data points for analysis: {len(processed_data)}")
    if not processed_data:
        print("No valid data found.")
        return None

    # ---------------------------
    # DataFrame & resample(30min)
    # ---------------------------
    df = pd.DataFrame(processed_data).set_index("start_time").sort_index()

    stats = df.resample("30min").agg(
        mu=("speed", "mean"),
        sigma=("speed", "std"),   # ddof=1 (표본 표준편차)
        n=("speed", "count"),
    )

    # μ ± σ / μ ± 2σ bands
    stats["lo_1sigma"] = stats["mu"] - stats["sigma"]
    stats["hi_1sigma"] = stats["mu"] + stats["sigma"]
    stats["lo_2sigma"] = stats["mu"] - 2 * stats["sigma"]
    stats["hi_2sigma"] = stats["mu"] + 2 * stats["sigma"]

    # 유효 구간만
    stats_filtered = stats.loc[stats["n"] > 0].copy()
    if len(stats_filtered) == 0:
        print("No data after filtering.")
        return None

    # ---------------------------
    # Process traffic data if provided
    # ---------------------------
    traffic_df = None
    if traffic_data_list:
        traffic_processed_data = []
        for item in traffic_data_list:
            if isinstance(item, dict):
                start_time = item.get("start_time")
                congestion = item.get("congestion_level")
            elif isinstance(item, (tuple, list)) and len(item) >= 2:
                start_time, congestion = item[0], item[1]
            else:
                continue

            if isinstance(start_time, (int, float)):
                start_datetime = base_time + timedelta(seconds=start_time)
            elif isinstance(start_time, datetime):
                start_datetime = start_time
            else:
                continue

            congestion_value = convert_traffic_level(congestion)
            if congestion_value is not None:
                traffic_processed_data.append({"start_time": start_datetime, "congestion": congestion_value})

        if traffic_processed_data:
            traffic_df = pd.DataFrame(traffic_processed_data).set_index("start_time").sort_index()

    # ---------------------------
    # Plot: Speed boxplot by time periods with optional traffic overlay
    # ---------------------------
    # NOTE: speed 단위가 m/s라면 3.6, 이미 km/h면 1.0
    K = 3.6

    # 원본 데이터를 30분 구간별로 그룹화하여 boxplot 생성
    df_plot = df.copy()
    df_plot["speed_kph"] = df_plot["speed"] * K
    df_plot["time_period"] = df_plot.index.floor("30min")

    # 각 시간 구간별 속도 데이터 수집
    time_periods = sorted(df_plot["time_period"].unique())
    speed_data = []
    period_labels = []

    for period in time_periods:
        period_speeds = df_plot[df_plot["time_period"] == period]["speed_kph"]
        if len(period_speeds) > 0:  # 데이터가 있는 구간만
            speed_data.append(period_speeds.values)
            period_labels.append(period.strftime("%H:%M"))

    if not speed_data:
        print("No data available for plotting.")
        return stats_filtered

    # Create subplot layout: main plot + optional traffic overlay
    if traffic_df is not None:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), height_ratios=[3, 1], sharex=True)
        main_ax = ax1
    else:
        fig, main_ax = plt.subplots(1, 1, figsize=(12, 6))
        ax2 = None

    # Boxplot 생성
    bp = main_ax.boxplot(speed_data, labels=period_labels, patch_artist=True,
                    showmeans=True, meanline=True,
                    boxprops=dict(facecolor='lightblue', alpha=0.7),
                    meanprops=dict(color='red', linewidth=2),
                    medianprops=dict(color='darkblue', linewidth=2),
                    whiskerprops=dict(color='black'),
                    capprops=dict(color='black'),
                    flierprops=dict(marker='o', markerfacecolor='red', markersize=3, alpha=0.5))

    # 제목/레이블
    main_ax.set_title(f"{title} - Speed Distribution (30-min intervals)", fontsize=14)
    if traffic_df is None:
        main_ax.set_xlabel("Time Period", fontsize=12)
    main_ax.set_ylabel("Speed (km/h)", fontsize=12)

    # 격자 및 축 포맷
    main_ax.grid(True, alpha=0.3, axis='y')

    # x축 레이블 회전 (너무 많으면)
    if len(period_labels) > 8 and traffic_df is None:
        plt.xticks(rotation=45)

    # 범례 추가 (평균선과 중앙값선 설명)
    main_ax.plot([], [], color='red', linewidth=2, label='Mean')
    main_ax.plot([], [], color='darkblue', linewidth=2, label='Median')
    main_ax.legend(loc='upper right')

    # ---------------------------
    # Add traffic congestion stacked area chart if data is available
    # ---------------------------
    if traffic_df is not None and ax2 is not None:
        # Resample traffic data to match time periods
        traffic_resampled = traffic_df.resample("30min").mean()
        
        # Create x positions for chart
        x_positions = list(range(len(period_labels)))
        traffic_values = []
        
        # Align traffic data with speed periods
        for period_str in period_labels:
            period_dt = datetime.strptime(period_str, "%H:%M").time()
            # Find matching traffic data for this period
            matching_traffic = None
            for traffic_time in traffic_resampled.index:
                if traffic_time.time().replace(second=0, microsecond=0) == period_dt:
                    matching_traffic = traffic_resampled.loc[traffic_time, 'congestion']
                    break
            
            if matching_traffic is not None:
                traffic_values.append(matching_traffic)
            else:
                traffic_values.append(0.2)  # Default to low traffic if no data
        
        # Define color mapping for traffic levels
        colors = {
            'low': '#1f77b4',      # Blue
            'medium': '#ff7f0e',   # Orange  
            'high': '#2ca02c',     # Green
            'jam': '#d62728'       # Red
        }
        
        # Create stacked area chart based on traffic levels
        for i, x_pos in enumerate(x_positions):
            congestion_level = traffic_values[i]
            
            # Determine traffic level and color
            if congestion_level <= 0.35:
                level = 'low'
                color = colors['low']
            elif congestion_level <= 0.6:
                level = 'medium' 
                color = colors['medium']
            elif congestion_level <= 0.8:
                level = 'high'
                color = colors['high']
            else:
                level = 'jam'
                color = colors['jam']
            
            # Draw bar for this time period (100% height, colored by level)
            ax2.bar(x_pos, 1.0, width=0.8, color=color, alpha=0.7, 
                   edgecolor='white', linewidth=0.5)
        
        ax2.set_ylabel("Traffic\nLevel", fontsize=10)
        ax2.set_xlabel("Time Period", fontsize=12)
        ax2.set_ylim(0, 1)
        ax2.set_xticks(x_positions)
        ax2.set_xticklabels(period_labels)
        ax2.grid(True, alpha=0.3, axis='x')
        
        # x축 레이블 회전
        if len(period_labels) > 8:
            plt.setp(ax2.xaxis.get_majorticklabels(), rotation=45)
        
        # Create legend with level colors
        from matplotlib.patches import Patch
        legend_elements = [
            Patch(facecolor=colors['low'], label='Low'),
            Patch(facecolor=colors['medium'], label='Medium'),
            Patch(facecolor=colors['high'], label='High'),
            Patch(facecolor=colors['jam'], label='Jam')
        ]
        ax2.legend(handles=legend_elements, loc='upper right', fontsize=8)
        
        # Remove y-axis ticks since it's categorical
        ax2.set_yticks([])

    plt.tight_layout()
    return stats_filtered


def analyze_vehicle_battery_consumption(
    vehicle_data_list, title="Vehicle Battery Consumption Analysis", traffic_data_list=None
):
    """Analyze battery consumption per vehicle and create visualization from raw data.

    Args:
        vehicle_data_list: List of dictionaries with keys:
                          - 'start_time': datetime or float (seconds from base time)
                          - 'battery_used': float (Ws - Watt Seconds consumed by vehicle)
                          OR list of tuples (start_time, battery_used)
        title: Title for the analysis plot
        traffic_data_list: Optional. List of dictionaries with keys:
                          - 'start_time': datetime or float (seconds from base time)
                          - 'congestion_level': string ("low"/"medium"/"high"/"jam", case-insensitive)
                                             OR float (0.0-1.0)

    Returns:
        pandas.DataFrame: 30-min binned stats (Wh-based μ/σ) filtered by n>0
    """

    print("=== Vehicle Battery Consumption Analysis ===")
    print(f"Input data points: {len(vehicle_data_list)}")

    if not vehicle_data_list:
        print("No data provided.")
        return None

    # ---------------------------
    # Normalize input data
    # ---------------------------
    processed_data = []
    base_time = datetime.now().replace(hour=0, minute=0, second=0, microsecond=0)

    for item in vehicle_data_list:
        if isinstance(item, dict):
            start_time = item.get("start_time")
            battery_used = item.get("battery_used")
        elif isinstance(item, (tuple, list)) and len(item) >= 2:
            start_time, battery_used = item[0], item[1]
        else:
            continue

        if isinstance(start_time, (int, float)):
            start_datetime = base_time + timedelta(seconds=start_time)
        elif isinstance(start_time, datetime):
            start_datetime = start_time
        else:
            continue

        if battery_used is not None and battery_used >= 0:
            # Ws → Wh
            battery_used_wh = battery_used / 3600.0
            processed_data.append({"start_time": start_datetime, "battery_used": battery_used_wh})

    print(f"Valid data points for analysis: {len(processed_data)}")
    if not processed_data:
        print("No valid data found.")
        return None

    # ---------------------------
    # DataFrame & resample(30min)
    # ---------------------------
    df = pd.DataFrame(processed_data).set_index("start_time").sort_index()

    stats = df.resample("30min").agg(
        mu=("battery_used", "mean"),
        sigma=("battery_used", "std"),   # ddof=1
        n=("battery_used", "count"),
    )

    # μ ± σ / μ ± 2σ (계산 유지: 리턴 포맷 호환)
    stats["lo_1sigma"] = stats["mu"] - stats["sigma"]
    stats["hi_1sigma"] = stats["mu"] + stats["sigma"]
    stats["lo_2sigma"] = stats["mu"] - 2 * stats["sigma"]
    stats["hi_2sigma"] = stats["mu"] + 2 * stats["sigma"]

    stats_filtered = stats.loc[stats["n"] > 0].copy()
    if len(stats_filtered) == 0:
        print("No data after filtering.")
        return None

    # ---------------------------
    # Plot: Battery consumption boxplot by time periods
    # ---------------------------
    # 원본 데이터를 30분 구간별로 그룹화하여 boxplot 생성
    df_plot = df.copy()
    df_plot["time_period"] = df_plot.index.floor("30min")

    # 각 시간 구간별 배터리 소모량 데이터 수집
    time_periods = sorted(df_plot["time_period"].unique())
    battery_data = []
    period_labels = []

    for period in time_periods:
        period_consumption = df_plot[df_plot["time_period"] == period]["battery_used"]
        if len(period_consumption) > 0:  # 데이터가 있는 구간만
            battery_data.append(period_consumption.values)
            period_labels.append(period.strftime("%H:%M"))

    if not battery_data:
        print("No data available for plotting.")
        return stats_filtered

    # ---------------------------
    # Process traffic data if provided
    # ---------------------------
    traffic_df = None
    if traffic_data_list:
        traffic_processed_data = []
        for item in traffic_data_list:
            if isinstance(item, dict):
                start_time = item.get("start_time")
                congestion = item.get("congestion_level")
            elif isinstance(item, (tuple, list)) and len(item) >= 2:
                start_time, congestion = item[0], item[1]
            else:
                continue

            if isinstance(start_time, (int, float)):
                start_datetime = base_time + timedelta(seconds=start_time)
            elif isinstance(start_time, datetime):
                start_datetime = start_time
            else:
                continue

            congestion_value = convert_traffic_level(congestion)
            if congestion_value is not None:
                traffic_processed_data.append({"start_time": start_datetime, "congestion": congestion_value})

        if traffic_processed_data:
            traffic_df = pd.DataFrame(traffic_processed_data).set_index("start_time").sort_index()

    # Create subplot layout: main plot + optional traffic overlay
    if traffic_df is not None:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), height_ratios=[3, 1], sharex=True)
        main_ax = ax1
    else:
        fig, main_ax = plt.subplots(1, 1, figsize=(12, 6))
        ax2 = None

    # Boxplot 생성
    bp = main_ax.boxplot(battery_data, labels=period_labels, patch_artist=True,
                    showmeans=True, meanline=True,
                    boxprops=dict(facecolor='lightcoral', alpha=0.7),
                    meanprops=dict(color='red', linewidth=2),
                    medianprops=dict(color='darkred', linewidth=2),
                    whiskerprops=dict(color='black'),
                    capprops=dict(color='black'),
                    flierprops=dict(marker='o', markerfacecolor='red', markersize=3, alpha=0.5))

    # 제목/레이블
    main_ax.set_title(f"{title} - Battery Consumption Distribution (30-min intervals)", fontsize=14)
    if traffic_df is None:
        main_ax.set_xlabel("Time Period", fontsize=12)
    main_ax.set_ylabel("Battery Consumption (Wh)", fontsize=12)

    # 격자 및 축 포맷
    main_ax.grid(True, alpha=0.3, axis='y')

    # x축 레이블 회전 (너무 많으면)
    if len(period_labels) > 8 and traffic_df is None:
        plt.xticks(rotation=45)

    # 범례 추가 (평균선과 중앙값선 설명)
    main_ax.plot([], [], color='red', linewidth=2, label='Mean')
    main_ax.plot([], [], color='darkred', linewidth=2, label='Median')
    main_ax.legend(loc='upper right')

    # ---------------------------
    # Add traffic congestion stacked area chart if data is available  
    # ---------------------------
    if traffic_df is not None and ax2 is not None:
        # Resample traffic data to match time periods
        traffic_resampled = traffic_df.resample("30min").mean()
        
        # Create x positions for chart
        x_positions = list(range(len(period_labels)))
        traffic_values = []
        
        # Align traffic data with battery consumption periods
        for period_str in period_labels:
            period_dt = datetime.strptime(period_str, "%H:%M").time()
            # Find matching traffic data for this period
            matching_traffic = None
            for traffic_time in traffic_resampled.index:
                if traffic_time.time().replace(second=0, microsecond=0) == period_dt:
                    matching_traffic = traffic_resampled.loc[traffic_time, 'congestion']
                    break
            
            if matching_traffic is not None:
                traffic_values.append(matching_traffic)
            else:
                traffic_values.append(0.2)  # Default to low traffic if no data
        
        # Define color mapping for traffic levels
        colors = {
            'low': '#1f77b4',      # Blue
            'medium': '#ff7f0e',   # Orange  
            'high': '#2ca02c',     # Green
            'jam': '#d62728'       # Red
        }
        
        # Create stacked area chart based on traffic levels
        for i, x_pos in enumerate(x_positions):
            congestion_level = traffic_values[i]
            
            # Determine traffic level and color
            if congestion_level <= 0.35:
                level = 'low'
                color = colors['low']
            elif congestion_level <= 0.6:
                level = 'medium' 
                color = colors['medium']
            elif congestion_level <= 0.8:
                level = 'high'
                color = colors['high']
            else:
                level = 'jam'
                color = colors['jam']
            
            # Draw bar for this time period (100% height, colored by level)
            ax2.bar(x_pos, 1.0, width=0.8, color=color, alpha=0.7, 
                   edgecolor='white', linewidth=0.5)
        
        ax2.set_ylabel("Traffic\nLevel", fontsize=10)
        ax2.set_xlabel("Time Period", fontsize=12)
        ax2.set_ylim(0, 1)
        ax2.set_xticks(x_positions)
        ax2.set_xticklabels(period_labels)
        ax2.grid(True, alpha=0.3, axis='x')
        
        # x축 레이블 회전
        if len(period_labels) > 8:
            plt.setp(ax2.xaxis.get_majorticklabels(), rotation=45)
        
        # Create legend with level colors
        from matplotlib.patches import Patch
        legend_elements = [
            Patch(facecolor=colors['low'], label='Low'),
            Patch(facecolor=colors['medium'], label='Medium'),
            Patch(facecolor=colors['high'], label='High'),
            Patch(facecolor=colors['jam'], label='Jam')
        ]
        ax2.legend(handles=legend_elements, loc='upper right', fontsize=8)
        
        # Remove y-axis ticks since it's categorical
        ax2.set_yticks([])

    plt.tight_layout()
    return stats_filtered