function alpha = newtonDamper(alpha, iter)
    if iter < 10
        alpha = 1.0;
    else
        alpha = alpha * 0.90;
    end

    if alpha < 0.1
        alpha = 0.1;
    end
end
