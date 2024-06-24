import random
import string

def mutstr(n):
    print("calling mutstr...(python)")
    ret = ''.join(random.sample(string.ascii_letters + string.digits, n))
    print("python is: {0}".format(ret))
    return ret

def mutnum(n):
    print("calling mutnum...(python)")
    ret = random.randint(0, n)
    print("python is: {0}".format(ret))
    return ret

if __name__ == "__main__":
    for i in range(10):
        print(mutstr(i*i))
        print(mutnum(i*i))