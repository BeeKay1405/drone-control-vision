import requests, statistics
times = []
for i in range(20):
    r = requests.get("http://192.168.137.252:9100/test_image/0")   # or <board-ip>
    times.append(r.json()["elapsed_ms"])
print(f"mean {statistics.mean(times):.1f} ms")
print(f"min {min(times):.1f}  max {max(times):.1f}  stdev {statistics.stdev(times):.1f}")