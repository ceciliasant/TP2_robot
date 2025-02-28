function A=Tlink(th, l, d, alpha)
A = hrotz(th) * htrans(l,0,d) * hrotx(alpha);