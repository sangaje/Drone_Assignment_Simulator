streams = 5
total_requests = 10_000
stride = 4096
cluster_size = 64               # 한 스트림이 같은 로컬 구간에서 보내는 요청 수
cluster_span = cluster_size * stride
base_gap = cluster_span * 16    # 스트림별 시작 위치 간격(4 MiB)
per_stream_count = [0] * streams

with open("test.txt", "w") as f:
    for i in range(total_requests):
        stream = i % streams
        seq = per_stream_count[stream]
        chunk = seq // cluster_size
        offset = seq % cluster_size
        base = stream * base_gap
        lba = base + chunk * cluster_span + offset * stride
        f.write(f"{i},{stream},W,{lba},{stride}\n")
        per_stream_count[stream] += 1
