function ddel_fi_pk1_pk2 = ddel_fi_del_p_k1_p_k2_corrected (vi, vj, vk, taui_0, unit_norm, A, k1, k2)
vk1 = zeros(3,1);
vk2 = zeros(3,1);
if k1 == 'i'
    vk1 = vi ;
elseif k1 == 'j'
    vk1 = vj ;
elseif k1 == 'k'
    vk1 = vk ;
else
    fprintf('error: k1 should be either i, j or k')
end
tk1 = cross(vk1, unit_norm); 

if k2 == 'i'
    vk2 = vi ;
elseif k2 == 'j'
    vk2 = vj ;
elseif k2 == 'k'
    vk2 = vk ;
else
    fprintf('error: k2 should be either i, j or k')
end
tk2 = cross(vk2, unit_norm); 

ddel_fi_pk1_pk2 =   (1/(4*A^2)).*( dot(taui_0,tk1).*(outer_prod(unit_norm,tk2) + outer_prod(tk2,unit_norm))) ;

