function sin_check(sen,line)
    if abs(sen)>1
        X=sprintf('ERROR:Line %d has absolute sin of %f',line,sen);
        disp(X);
end

