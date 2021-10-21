import math

if __name__ == '__main__':
    T = int(input())

    for t in range(T):
        (x1, y1, x2, y2) = map(int, list(input().split()))
        distance = round(math.sqrt(math.pow((x1 - x2), 2) + math.pow((y1 - y2), 2)), 2)
        print(f"Distance: {distance:.2f}")
