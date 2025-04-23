function [fig, ax] = makeFig(x_label,y_label,fontsize)
    fig = figure();
    ax = axes;
    fig.Position = [2000 100 900 700];
    set(ax,'FontWeight','bold','FontSize',fontsize,'FontName','OpenSans')
    set(fig,'Color','white')
    xlabel(x_label);
    ylabel(y_label);
    hold(ax,"on")
end