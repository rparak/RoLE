def fce_test(a, b, id):
    return{
        '0': lambda x, y: (x, y),
        '1': lambda x, y: (-x, -y)
    }[id](a, b)

print(fce_test(1, 2, '1'))