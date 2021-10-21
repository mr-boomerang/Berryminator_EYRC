T = int(input())

for t in range(T):
    n = int(input())
    bin_num = bin(n)[2:].zfill(8)
    print(bin_num)
    