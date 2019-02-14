# Building Test TFMicro Application

We will first build on x86 on a desktop or laptop to check that our model
works. The first model we will test is a small MNIST model trained in
Tensorflow and converted into TFLite.

## Building C++ TFLite Library

Make sure to clone tensorflow and follow their instructions to download
and set up bazel.

```bash
git clone https://www.github.com/tensorflow/tensorflow
cd tensorflow
bazel build //tensorflow/contrib/lite:framework
```

### Troubleshooting

Building simple C++ applications with TFMicro on an x86 laptop or desktop was
very annoying, here are some links that could be helpful.

* [Build TF-Life and Link](https://github.com/tensorflow/tensorflow/issues/16219)
