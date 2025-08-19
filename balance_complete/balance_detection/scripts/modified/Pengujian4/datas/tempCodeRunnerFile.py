# Menambahkan satu legenda di bagian bawah tengah
plt.figlegend(
    labels=file_names.values(),
    loc='lower center',
    ncol=5,
    fontsize=font_size,
    frameon=False
)