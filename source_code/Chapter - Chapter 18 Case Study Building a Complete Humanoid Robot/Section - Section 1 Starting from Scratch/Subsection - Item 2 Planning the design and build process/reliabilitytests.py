# Simple reliability and integration estimates (example values)
def system_reliability(reliabilities):
    # multiply component reliabilities -> series system
    r = 1.0
    for ri in reliabilities:
        r *= ri
    return r

def pairwise_tests(n):
    # number of pairwise integration tests
    return n*(n-1)//2

# Example usage
modules = {'mechanical':0.99, 'actuation':0.98, 'sensing':0.97, 'control':0.995}
print(system_reliability(modules.values()))  # overall R
print(pairwise_tests(len(modules)))         # integration count