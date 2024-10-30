function delfi_pk = delfi_by_delpk(taui_0, t_k, unit_norm, A)

% delfi_pk = ( (1/(2*A)).* (crossMat(v_k)*taui_0)) + ( (dot(taui_0, t_k)/(2*A)).* unit_norm );
delfi_pk = (dot(taui_0, t_k)/(2*A)).* unit_norm; % only term 2
delfi_pk = reshape(delfi_pk,[1,3]);
end