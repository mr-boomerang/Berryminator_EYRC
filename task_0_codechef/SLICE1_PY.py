T = int(input())

for t in range(T):
    length = int(input())
    L = list(map(int, list(input().split())))

    reversedL = list(reversed(L))
    print(' '.join([str(l) for l in reversedL]))

    thirds = [(l + 3) for l in L[3::3]]
    print(' '.join([str(l) for l in thirds]))

    fifths = [(l - 7) for l in L[5::5]]
    print(' '.join([str(l) for l in fifths]))

    three_to_seven = sum(L[3:7 + 1])
    print(three_to_seven)
