function DOFtheta= mapEdgetoDOFxi (edge_num,n_nodes,n_edges_thetas)

DOFtheta=n_nodes*3 + n_edges_thetas + edge_num;

end