import plotly.graph_objects as go

fig = go.Figure(
    data=go.Scattergeo(
        lat=[49.1154488073483, 54.868811366732665],
        lon=[-125.95696431913618, -163.35399921596127],
        mode="markers+lines",
        line=dict(width=2, color="blue"),
    )
)

fig.update_layout(
    title_text="Mock Global Path Plot",
    showlegend=True,
    geo=dict(
        showland=True,
        showcountries=True,
        showocean=True,
        countrywidth=0.5,
        landcolor="rgb(230, 145, 56)",
        lakecolor="rgb(0, 255, 255)",
        oceancolor="rgb(0, 255, 255)",
        projection=dict(type="orthographic", rotation=dict(lon=-100, lat=40, roll=0)),
        lonaxis=dict(showgrid=True, gridcolor="rgb(102, 102, 102)", gridwidth=0.5),
        lataxis=dict(showgrid=True, gridcolor="rgb(102, 102, 102)", gridwidth=0.5),
    ),
)

fig.show()
