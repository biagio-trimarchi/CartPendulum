function y = bernstein(t, n, k)
    % Compute the k^th Bernestein basis of order n
    y = nchoosek(n, k) * t ^ k * (1-t) ^ (n-k);
end