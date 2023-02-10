import pandas as pd
import plotly.graph_objects as go
import sys

sys.path.append('../scripts')

if __name__ == "__main__":

    argv = sys.argv
    log_name = argv[1]
    df = pd.read_csv(log_name)

    fig = go.Figure()
    fig.add_trace(go.Scattergl(
        x=df["x"],y=df["y"], 
        marker=dict(
            color=df["score"],
            colorbar=dict(title="Height[m]"),
            colorscale=["blue","yellow","red"],
        ),
        name="Position of LiDAR",
        mode="markers"
    ))
    fig.update_xaxes(linecolor='black',gridcolor='lightgray',mirror=True,title="x[m]")
    fig.update_yaxes(linecolor='black',gridcolor='lightgray',mirror=True,title="y[m]")
    fig.update_coloraxes(colorbar_title="TP",colorbar_dtick=1)
    fig.update_yaxes(scaleanchor = "x",scaleratio = 1)
    fig.update_layout(title="Height score trajectory")
    fig.update_layout(font={"size":20})
    fig.update_layout(legend=dict(
        x=0.01,
        y=0.99,
        xanchor='left',
        yanchor='top',
        orientation='h',
        bordercolor='black',
        borderwidth=1
    ))
    fig.update_layout(plot_bgcolor = "white")
    fig.write_image(log_name[0:-4]+"_traj.png")

    fig = go.Figure()
    fig.add_trace(go.Scattergl(
        x=df["relative_stamp"],
        y=df["score"],
        marker=dict(
            color="blue",
            size=3
        ),
        mode="lines")
    )
    fig.update_xaxes(linecolor='black',gridcolor='lightgray',mirror=True,title="time[s]")
    fig.update_yaxes(linecolor='black',gridcolor='lightgray',mirror=True,title="Height[m]")
    fig.update_coloraxes(colorbar_title="Height[m]",colorbar_dtick=1)
    fig.update_layout(title="Height score")
    fig.update_layout(font={"size":20})
    fig.update_layout(plot_bgcolor = "white")
    fig.write_image(log_name[0:-4]+".png")

