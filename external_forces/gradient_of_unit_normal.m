function del_nhat_del_qi = gradient_of_unit_normal(normal, q1,q2,q3, i)
if(i==1)
    del_nhat_del_qi = (1/norm(normal)^3)*(normal'*normal*eye(3) - normal*normal')*crossMat(q3-q2);

elseif(i==2)
    del_nhat_del_qi = (1/norm(normal)^3)*(normal'*normal*eye(3) - normal*normal')*crossMat(q1-q3);

elseif(i==3)
    del_nhat_del_qi = (1/norm(normal)^3)*(normal'*normal*eye(3) - normal*normal')*crossMat(q2-q1);
else
    error("i should be among 1, 2 or 3");
end
end