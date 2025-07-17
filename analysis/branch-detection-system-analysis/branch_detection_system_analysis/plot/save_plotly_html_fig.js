// Open browser console, type 'allow pasting' (with quotes), and then paste this function

(function() {
    const gd = document.querySelector(".js-plotly-plot");
    Plotly.toImage(gd, {
        format: 'svg',       // Or 'png'
        width: 1600,         // Your desired resolution
        height: 1200,
        scale: 2             // Increase scale for higher-res PNG
    }).then(function(url) {
        const a = document.createElement('a');
        a.href = url;
        a.download = 'highres_plot.svg';  // Change to .png if needed
        a.click();
    });
})();