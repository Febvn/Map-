import heapq


graph = {
    "Cilegon": {"Tangerang": 81},
    "Tangerang": {"Cilegon": 81, "Jakarta": 29},
    "Jakarta": {"Tangerang": 29, "Depok": 22, "Bekasi": 25},
    "Depok": {"Jakarta": 22, "Bogor": 44},
    "Bogor": {"Depok": 44, "Sukabumi": 57},
    "Sukabumi": {"Bogor": 57, "Bandung": 93},
    "Bekasi": {"Jakarta": 25, "Subang": 95, "Indramayu": 185},
    "Indramayu": {"Bekasi": 185, "Cirebon": 56},
    "Subang": {"Bekasi": 95, "Cirebon": 103},
    "Cirebon": {"Subang": 103, "Indramayu": 56, "Tegal": 71, "Bandung": 106},
    "Bandung": {"Sukabumi": 93, "Cirebon": 106, "Tasikmalaya": 63},
    "Tasikmalaya": {"Bandung": 63, "Cilacap": 96, "Purwokerto": 113},
    "Cilacap": {"Tasikmalaya": 96, "Purwokerto": 42},
    "Purwokerto": {"Cilacap": 42, "Tasikmalaya": 113, "Kebumen": 57, "Tegal": 65},
    "Kebumen": {"Purwokerto": 57, "Yogyakarta": 81},
    "Tegal": {"Cirebon": 71, "Pekalongan": 70, "Purwokerto": 65},
    "Pekalongan": {"Tegal": 70, "Semarang": 83},
    "Semarang": {"Pekalongan": 83, "Kudus": 60, "Ambarawa": 37},
    "Kudus": {"Semarang": 60, "Rembang": 62},
    "Rembang": {"Kudus": 62, "Tuban": 93},
    "Tuban": {"Rembang": 93, "Bojonegoro": 95},
    "Bojonegoro": {"Tuban": 95, "Ngawi": 103, "Surabaya": 111},
    "Surabaya": {"Bojonegoro": 111, "Sidoarjo": 35},
    "Sidoarjo": {"Surabaya": 35, "Probolinggo": 66, "Nganjuk": 118},
    "Probolinggo": {"Sidoarjo": 66, "Situbondo": 100, "Lumajang": 75},
    "Situbondo": {"Probolinggo": 100, "Banyuwangi": 88},
    "Banyuwangi": {"Situbondo": 88, "Jember": 100},
    "Jember": {"Banyuwangi": 100, "Lumajang": 65},
    "Lumajang": {"Jember": 65, "Probolinggo": 75, "Kepanjen": 116},
    "Kepanjen": {"Lumajang": 116, "Trenggalek": 114},
    "Trenggalek": {"Kepanjen": 114, "Pacitan": 108, "Ngawi": 86},
    "Pacitan": {"Trenggalek": 108, "Yogyakarta": 107},
    "Yogyakarta": {"Pacitan": 107, "Kebumen": 81, "Magelang": 40},
    "Magelang": {"Yogyakarta": 40, "Ambarawa": 35},
    "Ambarawa": {"Magelang": 35, "Semarang": 37},
    "Ngawi": {"Trenggalek": 86, "Surakarta": 72, "Bojonegoro": 103},
    "Surakarta": {"Ngawi": 72, "Magelang": 75},
}

# Fungsi untuk hitung total jarak
def total_distance(path):
    dist = 0
    for i in range(len(path)-1):
        dist += graph[path[i]][path[i+1]]
    return dist

# UCS
def ucs(start, goal):
    frontier = [(0, [start])]
    visited = set()
    while frontier:
        cost, path = heapq.heappop(frontier)
        node = path[-1]
        if node == goal:
            return path, cost
        if node not in visited:
            visited.add(node)
            for neighbor, c in graph.get(node, {}).items():
                if neighbor not in visited:
                    heapq.heappush(frontier, (cost + c, path + [neighbor]))
    return None, float("inf")

# IDS
def dls(node, goal, depth, path, visited):
    if node == goal:
        return path
    if depth <= 0:
        return None
    for neighbor in graph.get(node, {}):
        if neighbor not in visited:
            visited.add(neighbor)
            result = dls(neighbor, goal, depth - 1, path + [neighbor], visited)
            if result:
                return result
    return None

def ids(start, goal, max_depth=50):
    for depth in range(max_depth):
        visited = set([start])
        result = dls(start, goal, depth, [start], visited)
        if result:
            return result, total_distance(result)
    return None, -1


def gbfs(start, goal):
    heuristic = {node: 1 for node in graph}  
    frontier = [(heuristic[start], [start])]
    visited = set()
    while frontier:
        _, path = heapq.heappop(frontier)
        node = path[-1]
        if node == goal:
            return path, total_distance(path)
        if node not in visited:
            visited.add(node)
            for neighbor in graph.get(node, {}):
                if neighbor not in visited:
                    heapq.heappush(frontier, (heuristic.get(neighbor, 1), path + [neighbor]))
    return None, float("inf")

# A*
def astar(start, goal):
    heuristic = {node: 1 for node in graph}  
    frontier = [(heuristic[start], 0, [start])]
    visited = set()
    while frontier:
        est_total, cost, path = heapq.heappop(frontier)
        node = path[-1]
        if node == goal:
            return path, cost
        if node not in visited:
            visited.add(node)
            for neighbor, c in graph.get(node, {}).items():
                if neighbor not in visited:
                    g = cost + c
                    f = g + heuristic.get(neighbor, 1)
                    heapq.heappush(frontier, (f, g, path + [neighbor]))
    return None, float("inf")



def show_result(name, result):
    if result[0] is None:
        print(f"{name}: Tidak ditemukan")
    else:
        path, dist = result
        detail = " -> ".join(path)
        print(f"{name}: {detail}")
        print(f"Total jarak: {dist} km\n")

show_result("UCS", ucs("Cilegon", "Banyuwangi"))
show_result("IDS", ids("Cilegon", "Banyuwangi", max_depth=30))
show_result("GBFS", gbfs("Cilegon", "Banyuwangi"))
show_result("A*", astar("Cilegon", "Banyuwangi"))

