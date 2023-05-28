package com.example

import android.graphics.Bitmap
import android.graphics.BitmapFactory
import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.widget.TextView
import com.example.databinding.ActivityMainBinding

class MainActivity : AppCompatActivity() {

    private lateinit var binding: ActivityMainBinding

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        // Example of a call to a native method
        binding.sampleText.text = stringFromJNI()
        binding.btn2.setOnClickListener{
            val bitMap = BitmapFactory.decodeResource(resources,R.drawable.img);
            coverImg2Gray(bitMap);
            binding.img.setImageBitmap(bitMap)
        }

    }

    /**
     * A native method that is implemented by the 'example' native library,
     * which is packaged with this application.
     */
    external fun stringFromJNI(): String

    external fun coverImg2Gray(bitmap:Bitmap)

    companion object {
        // Used to load the 'example' library on application startup.
        init {
            System.loadLibrary("example")
        }
    }
}