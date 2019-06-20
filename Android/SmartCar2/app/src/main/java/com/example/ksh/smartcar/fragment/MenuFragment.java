package com.example.ksh.smartcar.fragment;

import android.app.Fragment;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;

import com.example.ksh.smartcar.interfaces.FragmentReplaceable;
import com.example.ksh.smartcar.R;

/**
 * Created by KSH on 2016-09-27.
 */

public class MenuFragment extends Fragment {
    private Button buttonManual;
    private Button buttonAuto;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        View rootView = inflater.inflate(R.layout.activity_menu, container, false);
        buttonManual = (Button) rootView.findViewById(R.id.buttonManual);
        buttonAuto = (Button) rootView.findViewById(R.id.buttonAuto);

        buttonManual.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                ((FragmentReplaceable)getActivity()).replaceFragment(1);
            }
        });

        buttonAuto.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                ((FragmentReplaceable)getActivity()).replaceFragment(2);
            }
        });
        return rootView;
    }
}
