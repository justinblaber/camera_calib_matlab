function kernel = gauss_kernel(p, cov, xs, ys, s)

    kernel = mvnpdf([xs ys], p, cov);
    kernel_gauss = reshape(kernel_gauss,[s s]);
end