function PlotUpdate()   
    x = 0:.1:8;
    y = sin(x);
    h = plot(x,y);         
    set(h,'YDataSource','y')
    set(h,'XDataSource','x')
    theAxes = axis;
    fmat = moviein(2000);
    for ii =1:10
        for jj=1:2
        y=(x/ii*jj);
        refreshdata(h,'caller');
        axis(theAxes);
        fmat(:,ii)=getframe;
        end
    end
    movie(fmat)
end