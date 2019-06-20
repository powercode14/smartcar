package com.example.ksh.smartcar.fragment;


import android.app.Fragment;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;

import com.example.ksh.smartcar.MainActivity;
import com.example.ksh.smartcar.R;

import android.net.Uri;
import android.widget.ImageView;
import android.widget.Toast;
import android.widget.VideoView;

public class VideoFragment extends Fragment {

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        View view = inflater.inflate(R.layout.activity_video, container, false);
        String uri = "rtsp://" + MainActivity.getServerIP() + ":8555/unicast";
        Toast.makeText(getActivity(), uri, Toast.LENGTH_SHORT).show();
        VideoView v = (VideoView) view.findViewById( R.id.videoView2);
        v.setVideoURI(Uri.parse(uri));
        v.requestFocus();
        v.start();
        return view;
    }
}
