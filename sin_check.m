function sin_check(sen,i)
    if abs(sen)>1
        X=sprintf('ERROR:Teta(%d) has absolute sin of %f',i,sen);
        disp(X);
end

