// server.js
const express = require('express');
const app = express();
const port = 80;

let sensorData = 0;

app.get('/update', (req, res) => {
    if (req.query.value) {
        sensorData = req.query.value;
        console.log(`Received data: ${sensorData}`);
        res.sendStatus(200);
    } else {
        res.sendStatus(400);
    }
});

app.get('/', (req, res) => {
    res.send(`
        <!DOCTYPE html>
        <html>
        <head>
            <title>Sensor Data</title>
            <meta http-equiv="refresh" content="1">
        </head>
        <body>
            <h1>Sensor Data: ${sensorData} cm</h1>
        </body>
        </html>
    `);
});

app.listen(port, () => {
    console.log(`Server running at http://localhost:${port}/`);
});