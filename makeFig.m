function [fig, ax] = makeFig(x_label,y_label,fontsize)
    fig = figure();
    ax = axes;
    fig.Position = [2000 100 900 500];
    set(ax,'FontWeight','bold','FontSize',fontsize,'FontName','Times')
    set(fig,'Color','white')
    xlabel(x_label);
    ylabel(y_label);
    hold(ax,"on")
end