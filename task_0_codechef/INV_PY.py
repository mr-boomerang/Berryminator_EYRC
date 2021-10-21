def adding(item, qty, lab):
    if item not in lab:
        lab[item] = qty
        print("ADDED Item " + item)
        return
    lab[item] += qty
    print("UPDATED Item " + item)

def deleting(item, qty, lab):
    if item not in lab:
        print("Item " + item + " does not exist")
        return
    
    if (lab[item] < qty):
        print("Item " + item + " could not be DELETED")
        return
    
    lab[item] -= qty
    print("DELETED Item " + item)
   

if __name__ == "__main__":
    T = int(input())
    for t in range(T):
        lab = {}

        N = int(input())
        for n in range(N):
            (item, qty) = input().split()
            qty = int(qty)
            lab[item] = qty

        M = int(input())
        for m in range(M):
            (op, item, qty) = input().split()
            qty = int(qty)

            if op == "ADD":
                adding(item, qty, lab)
            elif op == "DELETE":
                deleting(item, qty, lab)
            
        total = int(sum(lab.values()))
        print("Total Items in Inventory: " + str(total))