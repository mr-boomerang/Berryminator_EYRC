T = int(input())

for t in range(T):
    N = int(input())
    
    students = []
    for n in range(N):
        (name, score) = input().split()
        students.append((name, float(score)))
    
    students.sort(key=lambda student: student[1], reverse=True)
    
    toppers = []
    i = 0
    while(students[i][1] >= students[0][1]):
        toppers.append(students[i][0])
        i+=1
    
    toppers.sort()
    for topper in toppers:
        print(topper)
