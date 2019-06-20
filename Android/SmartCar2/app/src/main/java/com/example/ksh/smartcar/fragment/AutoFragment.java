package com.example.ksh.smartcar.fragment;

import android.app.Activity;
import android.app.Fragment;
import android.content.Context;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.ListView;


import com.example.ksh.smartcar.MainActivity;
import com.example.ksh.smartcar.list.ListItem;
import com.example.ksh.smartcar.list.ListItemAdapter;
import com.example.ksh.smartcar.R;
import com.example.ksh.smartcar.Applications;

import java.util.ArrayList;

public class AutoFragment extends Fragment {
    private OnFragmentInteractionListener mListener;
    private ArrayList<ListItem> items;
    private Applications applications;
    ListView listView;
    private ListItemAdapter adapter;
    private Button buttonClear;

    Activity activity;

    public OnFragmentInteractionListener getmListener() {
        return mListener;
    }

    @Override
    public void onAttach(Context context) {
        super.onAttach(context);
        if (context instanceof OnFragmentInteractionListener) {
            mListener = (OnFragmentInteractionListener) context;
        } else {
            throw new RuntimeException(context.toString()
                    + " must implement OnFragmentInteractionListener");
        }
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        View rootview = inflater.inflate(R.layout.activity_auto, container, false);
        activity = getActivity();
        buttonClear = (Button) rootview.findViewById(R.id.buttonClear);
        applications = (Applications) getActivity().getApplicationContext();
        listView = (ListView) rootview.findViewById(R.id.listView);
        applications.setListView(listView);
        applications = (Applications) rootview.getContext().getApplicationContext();
        applications.setAutoFragment(this);
        items = applications.getListItems();
        adapter = new ListItemAdapter(getActivity(), R.layout.list_item , items);
        applications.setListItemAdapter(adapter);
        listView.setAdapter(adapter);
        buttonClear.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                clearList();
            }
        });
        return rootview;
    }

    @Override
    public void onDetach() {
        super.onDetach();
        mListener = null;
    }

    public interface OnFragmentInteractionListener {
        void updateList();
    }

    public void updateList(){
        if(mListener != null) {
            adapter = new ListItemAdapter(getActivity(), R.layout.list_item , items);
            listView.setAdapter(adapter);
        }
    }

    public void clearList(){
        items.clear();
        adapter = new ListItemAdapter(getActivity(), R.layout.list_item , items);
        listView.setAdapter(adapter);
    }
}