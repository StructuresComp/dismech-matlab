function [rod_data,shell_data] = logDataForRendering(dof_with_time, MultiRod, Nsteps)

rod_data = zeros(MultiRod.n_rod_nodes*Nsteps,4);
for i=1:Nsteps
    for j=1:MultiRod.n_rod_nodes
        rod_data((i-1)*MultiRod.n_rod_nodes+j,:) = [dof_with_time(1,i), dof_with_time(1+mapNodetoDOF(j),i)'];
    end
end

shell_data = zeros(3*MultiRod.n_faces*Nsteps,3);
for i=1:Nsteps
    for j=1:MultiRod.n_faces
        n1 = MultiRod.face_nodes_shell(j,1);
        n2 = MultiRod.face_nodes_shell(j,2);
        n3 = MultiRod.face_nodes_shell(j,3);
        shell_data((i-1)*3*MultiRod.n_faces+3*j-2: (i-1)*3*MultiRod.n_faces+3*j,:) = [
            dof_with_time(1+mapNodetoDOF(n1),i)';
            dof_with_time(1+mapNodetoDOF(n2),i)';
            dof_with_time(1+mapNodetoDOF(n3),i)'];
    end
end
writematrix(rod_data,'rawDataRod.txt', 'Writemode', "overwrite")
writematrix(shell_data,'rawDataShell.txt', 'Writemode', "overwrite")