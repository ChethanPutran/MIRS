def get_pivot(arr, low, high):
    i = low - 1
    pivot = arr[high]

    for j in range(low, high):
        if (arr[j] <= pivot):
            i += 1
            arr[i], arr[j] = arr[j], arr[i]
    arr[i+1], arr[high] = arr[high], arr[i+1]
    return i+1


def quick(arr, low=0, high=None):
    if high is None:
        high = len(arr)-1

    if low < high:
        pivot_idx = get_pivot(arr, low, high)
        quick(arr, low, pivot_idx-1)
        quick(arr, pivot_idx+1, high)


arr = [7, 3, 4, 5, 9, 1, 2]
quick(arr)
print(arr)


def dijec(graph, n, src):
    visited = [0]*n
    dist = [float("inf")]*n
    dist[src] = 0

    u = src
    visited[src] = 1
    for _ in range(n):
        # Get the min dist unvisited vertex from list
        min_dist = float("inf")
        for i in range(n):
            if (not visited[i] and dist[i] < min_dist):
                u = i
                min_dist = dist[i]

        for col in range(n):
            if not visited[col] and graph[u][col] != 0:
                if dist[col] > graph[u][col] + dist[u]:
                    dist[col] = graph[u][col] + dist[u]

        visited[u] = True


def prims(n):
    in_mst = [0]*n
    values = [0]
