package com.wizzy.frontend;

import androidx.appcompat.app.AppCompatActivity;

import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.View;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.Socket;

import static java.lang.Boolean.TRUE;

public class MainActivity extends AppCompatActivity {
    // Networking
    private Socket          mySocket;
    private PrintWriter     outToServer;
    private BufferedReader  inFromServer;
    boolean isAsyncTaskRunning;

    // GUI
    private EditText    myEditText;
    TextView            myTextView;
    String              msgToServer = "";

    public MainActivity() {
        mySocket    = null;
        outToServer = null;
        inFromServer= null;
        isAsyncTaskRunning = false;
    }

    // Periodic operation
    Handler handler = new Handler();
    
    private Runnable runnableCode = new Runnable() {
        @Override
        public void run() {
            if (isAsyncTaskRunning == false) {
                // Do something here on the main thread
                Log.e("Handlers", "Called on main thread");

                // Async class
                myCommTask mt = new myCommTask();
                mt.execute();
            }
            else
            {
                Log.e("Handlers", "Async has not finished running");
            }

            handler.postDelayed(this, 1000);
        }
    };


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        myEditText = (EditText) findViewById(R.id.editText);
        myTextView = (TextView) findViewById(R.id.textView);

        myTextView.setText("Init");

    }


    public void send_text (View v)
    {
        msgToServer = myEditText.getText().toString();

        // Start the initial runnable task by posting through the handler
        handler.post(runnableCode);

        Toast.makeText(getApplicationContext(), "Activated Periodic Comm", Toast.LENGTH_LONG).show();
    }


    class myCommTask extends AsyncTask <Void, Void, String>  {

        @Override
        protected String doInBackground(Void... voids) {
            isAsyncTaskRunning = true;

            if(android.os.Debug.isDebuggerConnected()) {
                android.os.Debug.waitForDebugger();
            }

            String msgFromServer = "";

            try {
                if (mySocket == null) {
                    mySocket     = new Socket("192.168.43.10", 5000);
                    outToServer  = new PrintWriter(mySocket.getOutputStream());
                    inFromServer = new BufferedReader(new InputStreamReader(mySocket.getInputStream()));
                }

                outToServer.write(msgToServer);
                outToServer.flush();

                msgFromServer = inFromServer.readLine();
                // Currently not closing socket. This is a problem if server socket closes and open again,
                // then this tcp client still have the stale socket, and no communication will happen.
                // Thus closing and opening the napplication is now needed
//                outToServer.close();

            } catch (IOException e) {
                e.printStackTrace();
            }

            return msgFromServer;
        }

        @Override
        protected void onPostExecute(String response) {
            isAsyncTaskRunning = false;

            if(android.os.Debug.isDebuggerConnected()) {
                android.os.Debug.waitForDebugger();
            }

            if (response != null) {
                myTextView.setText("Connected");
                Toast.makeText(getApplicationContext(), response, Toast.LENGTH_LONG).show();
            }
            else
            {
                myTextView.setText("Error Connection");
            }
        }
    }
}
