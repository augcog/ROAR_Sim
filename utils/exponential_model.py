import numpy as np

def Sk(S_k_1, y_k, y_k_1, x_k, x_k_1):
    return S_k_1 + 0.5 * (y_k + y_k_1) * (x_k - x_k_1)

def SSk(SS_k_1, S_k, S_k_1, x_k, x_k_1):
    return SS_k_1 + 0.5 * (S_k + S_k_1) * (x_k - x_k_1)

def S(x, y):
    ret_S = [0]
    for k in range(1, len(x)):
        S_k = Sk(ret_S[k-1], y[k], y[k-1], x[k], x[k-1])
        ret_S.append(S_k)
    return ret_S

def SS(s, x):
    ret_SS = [0]
    for k in range(1, len(x)):
        SS_k = SSk(ret_SS[k-1], s[k], s[k-1], x[k], x[k-1])
        ret_SS.append(SS_k)
    return ret_SS

def F1(SS_k, y_k):
    return SS_k / y_k

def F2(S_k, y_k):
    return S_k / y_k

def F3(x_k, y_k):
    return (x_k ** 2) / y_k

def F4(x_k, y_k):
    return x_k / y_k

def F5(y_k):
    return 1 / y_k

def construct_f(a, b, c, p, q):
    def f(x):
        # print(type(a), type(b), type(c), type(p), type(q))
        return a + b*np.exp(p*x) + c*np.exp(q*x)
    return f

def fit(x, y):
    Sxy = S(x,y)
    SSxy = SS(Sxy, x)
    F1xy = F1(SSxy, y)
    F2xy = F2(Sxy, y)
    F3xy = F3(x, y)
    F4xy = F4(x, y)
    F5xy = F5(y)
    F = np.array([F1xy, F2xy, F3xy, F4xy, F5xy])
    f = np.array([np.sum(F1xy), np.sum(F2xy), np.sum(F3xy), 
                  np.sum(F4xy), np.sum(F5xy)])
    F = F @ F.T
    A, B, C, D, E = np.linalg.inv(F) @ f
    pre_sqrt = np.clip(B**2 + 4*A, 0, np.inf) # edits 1
    
    p = 0.5 * (B + np.sqrt(pre_sqrt))
    q = 0.5 * (B - np.sqrt(pre_sqrt))
    G1 = 1 / y
    G2 = np.exp(p*x) / y
    G3 = np.exp(q*x) / y
    G = np.array([G1, G2, G3])
    G = G @ G.T
    g = np.array([np.sum(G1), np.sum(G2), np.sum(G3)])
    a, b, c = np.linalg.pinv(G) @ g # edits 2
    return a, b, c, p, q