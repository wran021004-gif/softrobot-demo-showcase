"""ColorBrewer BuGn n=3 and categorical Dark2; no red/green-only encoding."""
BUGN=['#E5F5F9','#99D8C9','#2CA25F']
DARK2=['#1b9e77','#d95f02','#7570b3','#666666']
INK='#252525';FLOOR='#F0F0F0' # ColorBrewer Greys neutrals
def rgba(h,alpha=1.):return [int(h[i:i+2],16)/255 for i in [1,3,5]]+[alpha]
def scientific_style():
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':9,'axes.labelsize':9,'axes.titlesize':10,'legend.fontsize':7,'axes.spines.top':False,'axes.spines.right':False,'lines.linewidth':1.35,'svg.fonttype':'none','axes.labelcolor':INK,'xtick.color':INK,'ytick.color':INK})
