function proSingolarita(J)

syms aa bb rho 

detJ=det(J);
solve(detJ==0,[aa bb rho])

end